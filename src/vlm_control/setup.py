from setuptools import setup

package_name = 'vlm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lchin',
    maintainer_email='lchin@todo.todo',
    description='VLM 视觉避障的决策与执行层',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoidance_controller = vlm_control.controller_node:main',
            'imu_state_node = vlm_control.imu_node:main',
        ],
    },
)
