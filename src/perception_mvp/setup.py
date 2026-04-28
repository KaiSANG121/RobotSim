from setuptools import setup

package_name = 'perception_mvp'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaisang',
    maintainer_email='kaisang@example.com',
    description='RGB-D perception, state machine, and MoveIt2 execution for Alicia-D grasping simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_perception_node = perception_mvp.color_perception_node:main',
            'move_to_pregrasp_node = perception_mvp.move_to_pregrasp_node:main',
            'task_state_machine_node = perception_mvp.task_state_machine_node:main',
        ],
    },
)
