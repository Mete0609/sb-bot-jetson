import os
import time
import numpy as np
import pyaudio
import wave
from dashscope.audio.asr import Recognition
from http import HTTPStatus
from dashscope import MultiModalConversation
import json
import dashscope
from dashscope.audio.tts_v2 import *
import os
from playsound import playsound  # 导入 playsound 库用于播放音频
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ----------- 参数设置 -----------
VOLUME_THRESHOLD = 800    # 音量阈值
SILENCE_DURATION = 2       # 静音持续时间阈值 (秒)
RECORD_SECONDS_LIMIT = 60  # 最长录音时间（避免无限录音）
WAV_OUTPUT_FILENAME = 'asr_input.wav'
SAMPLE_RATE = 16000
CHANNELS = 1
CHUNK = 1024
FORMAT = pyaudio.paInt16

# ----------- 音量计算函数 -----------
def calculate_volume(audio_data):
    audio_np = np.frombuffer(audio_data, dtype=np.int16)
    if len(audio_np) == 0:
        return 0.0

    audio_np = audio_np.astype(np.float32)  # ⚡注意，一定要先转float32！

    power = np.mean(audio_np ** 2)

    if np.isnan(power) or np.isinf(power) or power < 0:
        return 0.0

    rms = np.sqrt(power)
    return rms

# ----------- 开始录音函数 -----------
def record_audio_when_voice_detected():
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=SAMPLE_RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    print("等待音量达到阈值开始录音...")

    frames = []
    recording = False
    silence_start = None

    while True:
        data = stream.read(CHUNK, exception_on_overflow=False)
        volume = calculate_volume(data)
        print(f"当前音量：{volume}")

        current_time = time.time()

        if volume > VOLUME_THRESHOLD:
            if not recording:
                print("检测到声音，开始录音！")
                recording = True
            frames.append(data)
            silence_start = None  # 音量大了就清除静音计时
        else:
            if recording:
                if silence_start is None:
                    silence_start = current_time  # 第一次进入静音，开始计时
                    print("进入静音检测...")
                elif (current_time - silence_start) > SILENCE_DURATION:
                    print("检测到静音超过2秒，停止录音。")
                    break  # 超过2秒静音，停止录音

    print("保存录音到 asr_input.wav")
    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(WAV_OUTPUT_FILENAME, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(SAMPLE_RATE)
    wf.writeframes(b''.join(frames))
    wf.close()


# ----------- 调用ASR识别函数 -----------
def recognize_audio():
    recognition = Recognition(
        model='paraformer-realtime-v2',
        format='wav',
        sample_rate=16000,
        language_hints=['zh', 'en'],
        callback=None
    )
    result = recognition.call(WAV_OUTPUT_FILENAME)

    if result.status_code == HTTPStatus.OK:
        print('识别结果：')
        sentences = result.get_sentence()
        if sentences and isinstance(sentences, list):
            for sentence in sentences:
                if 'text' in sentence:
                    print(sentence['text'])
                    return sentence['text']
                else:
                    print("该句无text字段")
        else:
            print("未返回有效识别内容。")
    else:
        print('Error: ', result.message)

def LLM_sent_message(sentence):
    messages = [
        {
            "role": "system",
            "content": [
                {
                    "text": """
                        你是一个家庭陪护机器人，名字是傻宝。你根据用户的信息，严格输出标准JSON格式，遵循以下要求：
                        - 属性名和值必须使用双引号("")。
                        - 每个对象最后一项后不得添加逗号。
                        - 不能添加注释或额外解释。
                        - <>里的内容使用中文，最大长度不超过100字
                        {
                            "control": {
                                "action": "<动作类型>",
                                "parameters": {
                                    "position": "<位置>",
                                    "target":<目标>
                                }
                            },
                            "response": {
                                "message": "<信息>"
                            }
                        }

                        例1：
                        "我想让你去书桌拿瓶水"
                        输出：
                        {
                            "control": {
                                "action": "move",
                                "parameters": {
                                    "position": "书桌",
                                    "target":"bottle"
                                }
                            },
                            "response": {
                                "message": "好的我正在前往。"
                            }
                        }
                        例2：
                        "请帮我去餐桌上拿纸巾"
                        输出：
                        {
                            "control": {
                                "action": "move",
                                "parameters": {
                                    "position": "餐桌",
                                    "target":"tissue"
                                }
                            },
                            "response": {
                                "message": "好的我正在前往。"
                            }
                        }
                    """
                }
            ]
        },
        {
            'role': 'user',
            'content': [
                # {'image': 'image_path'},  # 将图像路径传递给模型以进行识别
                # {'text': '请帮我去厨房拿瓶水。'}
                {'text': sentence}
            ]
        }
    ]
    response = MultiModalConversation.call(
        # 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx"
        api_key=os.getenv("DASHSCOPE_API_KEY"), 
        model='qwen-vl-max-latest',
        messages=messages)
    print(response["output"]["choices"][0]["message"].content[0]["text"])
    return response["output"]["choices"][0]["message"].content[0]["text"]

def LLM_text_respone(llm_output_text):
    try:
        llm_result = json.loads(llm_output_text)

        # 2. 提取 response.message
        if 'response' in llm_result and 'message' in llm_result['response']:
            message_text = llm_result['response']['message']
            return message_text
            # print("提取到的机器人回复是：", message_text)
        else:
            print("解析失败：找不到 response 或 message 字段。")

    except json.JSONDecodeError as e:
        print("解析JSON失败：", e)

def LLM_text_control(input_dict):
    try:
        if isinstance(input_dict, str):
            input_dict = json.loads(input_dict)  # 将字符串转换为字典
        
        # 输入类型检查
        if not isinstance(input_dict, dict):
            return "Error: 输入必须是字典类型！"
        
        # 获取控制部分
        control = input_dict.get("control", {})

        # 构造新的字典，返回控制部分
        result = {
            "control": control
        }

        # 转换为 JSON 字符串
        return json.dumps(result, ensure_ascii=False)

    except Exception as e:
        return f"Error: {e}"


# 文本转语音
def Text_To_Speech(LLM_sentence):
    model = "cosyvoice-v2"
    # 音色
    voice = "longxiaoxia_v2"

    # 实例化SpeechSynthesizer，并在构造方法中传入模型（model）、音色（voice）等请求参数
    synthesizer = SpeechSynthesizer(model=model, voice=voice)

    # 发送待合成文本，获取二进制音频
    # audio = synthesizer.call("你好，我是傻宝，你的智能陪护助手，请问有能帮得上忙的吗？")
    audio = synthesizer.call(LLM_sentence)
    # 打印请求的延迟等信息
    print('[Metric] requestId: {}, first package delay ms: {}'.format(
        synthesizer.get_last_request_id(),
        synthesizer.get_first_package_delay()))

    # 将音频保存至本地
    audio_path = 'output.mp3'
    with open(audio_path, 'wb') as f:
        f.write(audio)

    # 播放保存的音频
    playsound(audio_path)  # 自动播放音频

def app_bridge_control(llm_output_text, node):
    try:
        llm_result = json.loads(llm_output_text)

        control_part = llm_result.get('control', None)

        if control_part is None:
            print("❌ 解析失败：没有找到 control 部分！")
            return False

        msg = String()
        msg.data = json.dumps(control_part, ensure_ascii=False)
        node.publisher_.publish(msg)
        node.get_logger().info(f'🚀 已发送控制指令: {msg.data}')
        return True

    except json.JSONDecodeError as e:
        print(f"❌ JSON解析错误: {e}")
    except Exception as e:
        print(f"❌ 出现异常: {e}")

    return False

def main(args=None):
    rclpy.init(args=args)

    from rclpy.node import Node
    class AppBridgePublisher(Node):
        def __init__(self):
            super().__init__('app_bridge_publisher')
            self.publisher_ = self.create_publisher(String, '/app_navigation_target', 10)

    node = AppBridgePublisher()

    try:
        while True:
            record_audio_when_voice_detected()
            sentence = recognize_audio()
            llm_output_text = LLM_sent_message(sentence)
            TTS_input = LLM_text_respone(llm_output_text)
            # print(f"收到的llm_output_text: {llm_output_text}")
            Text_To_Speech(TTS_input)
            control_text = LLM_text_control(llm_output_text)
            print(f"收到的control_text: {control_text}")
            app_bridge_control(control_text, node)  # 传node进去！

    except Exception as e:
        print(f"❌ 主程序运行异常: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
