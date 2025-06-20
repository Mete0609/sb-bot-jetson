from setuptools import find_packages, setup

package_name = 'joint_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy', 'pyserial'],
    zip_safe=True,
    maintainer='sb-bot',
    maintainer_email='1462150552@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller_node = joint_controller.joint_controller_node:main'
        ],
    },
)
