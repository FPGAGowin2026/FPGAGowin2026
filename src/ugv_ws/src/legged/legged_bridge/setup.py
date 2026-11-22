from setuptools import find_packages, setup

setup(
    name='legged_bridge', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/legged_bridge']),
                ('share/legged_bridge', ['package.xml', 'README.md'])],
    install_requires=['setuptools'], zip_safe=False,
    maintainer='Robot project', maintainer_email='robot@example.com',
    description='ACG720 UDP control, sensors and camera calibration', license='Apache-2.0',
    entry_points={'console_scripts': [
        'acg720_bridge = legged_bridge.acg720_bridge:main',
        'acg720_camera_calibration = legged_bridge.acg720_calibration:main',
    ]},
)
