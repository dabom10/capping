from setuptools import find_packages, setup

package_name = 'dsr_joint_sub'

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
    maintainer='dongchanseo',
    maintainer_email='dongchanseo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_state_sub = dsr_joint_sub.joint_state_sub:main',
            'robot_disconnection_sub = dsr_joint_sub.robot_disconnection_sub:main',
        ],
    },
)
