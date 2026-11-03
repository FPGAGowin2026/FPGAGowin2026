from setuptools import find_packages, setup

package_name = 'swarm_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='aa',
    maintainer_email='aa@example.com',
    description='Wheel search result handoff candidate',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'handoff_candidate = swarm_manager.handoff_candidate:main',
        'target_projection = swarm_manager.target_projection:main',
        'target_to_legged = swarm_manager.target_to_legged:main',
        'relay_goal_gateway = swarm_manager.relay_goal_gateway:main',
    ]},
)
