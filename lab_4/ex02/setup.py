from setuptools import setup, find_packages

package_name = 'turtle_tf2_lab4_ex02'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtle_carrot.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/carrot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Turtle TF2 broadcaster and listener with carrot frame for lab4 ex02',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf2_broadcaster = turtle_tf2_lab4_ex02.turtle_tf2_broadcaster:main',
            'turtle2_tf2_broadcaster = turtle_tf2_lab4_ex02.turtle2_tf2_broadcaster:main',
            'carrot_tf2_broadcaster = turtle_tf2_lab4_ex02.carrot_tf2_broadcaster:main',
            'turtle_tf2_listener = turtle_tf2_lab4_ex02.turtle_tf2_listener:main',
        ],
    },
)
