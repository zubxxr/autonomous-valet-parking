from setuptools import find_packages, setup

package_name = 'avp_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_avp_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zubair Islam',
    maintainer_email='zubxxr@gmail.com',
    description='Top-level orchestrator node for AVP system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Main AVP node
            'avp_node = avp_node.avp_node:main',

            # Tools
            'generate_goal_pose_command = avp_node.tools.generate_goal_pose_command:main',
            'generate_initial_pose_command = avp_node.tools.generate_initial_pose_command:main',

            # Debugging
            'publish_parking_spots = avp_node.debugging.publish_parking_spots:main',
        ],
    },
)
