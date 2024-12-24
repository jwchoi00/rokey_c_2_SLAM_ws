from setuptools import find_packages, setup

package_name = 'turtlebot4_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='g1',
    maintainer_email='jwchoi0017@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot4_python = turtlebot4_python.turtlebot4_python:main',
            'turtlebot4_wall_follow = turtlebot4_python.turtlebot4_wall_follow:main',
            'turtlebot4_explorer = turtlebot4_python.turtlebot4_explorer:main',
            'turtlebot4_explorer_ver2 = turtlebot4_python.turtlebot4_explorer_ver2:main',
            'turtlebot4_explorer_ver3 = turtlebot4_python.turtlebot4_explorer_ver3:main',
            'turtlebot4_explorer_ver4 = turtlebot4_python.turtlebot4_explorer_ver4:main',
            'turtlebot4_map_save = turtlebot4_python.turtlebot4_map_save:main',
            'turtlebot4_map_astar = turtlebot4_python.turtlebot4_map_astar:main',
            'turtlebot4_map_astar_ver2 = turtlebot4_python.turtlebot4_map_astar_ver2:main',
            'turtlebot4_map_astar_ver4 = turtlebot4_python.turtlebot4_map_astar_ver4:main',
            'turtlebot4_go_to_goal = turtlebot4_python.turtlebot4_go_to_goal:main',
            'turtlebot4_go_to_goal_ver2 = turtlebot4_python.turtlebot4_go_to_goal_ver2:main',
        ],
    },
)
