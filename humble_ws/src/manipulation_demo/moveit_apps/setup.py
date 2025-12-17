from setuptools import find_packages, setup

package_name = 'moveit_apps'

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
    maintainer='alexthunderrex',
    maintainer_email='alex.kalm.dev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_and_place = moveit_apps.pick_and_place:main',
            'move_to_pose = moveit_apps.move_to_pose_node:main'
        ],
    },
)
