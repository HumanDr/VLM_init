from setuptools import setup

package_name = 'vlm_perception'

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
    description='相机 → Qwen-VL 语义避障感知层',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlm_node = vlm_perception.vlm_node:main',
            'depth_sectors_node = vlm_perception.depth_sectors_node:main',
            'mock_vlm_server = vlm_perception.mock_vlm_server:main',
            'fake_camera = vlm_perception.fake_camera:main',
            'scan_sectors_node = vlm_perception.scan_sectors_node:main',
        ],
    },
)
