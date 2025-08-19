from setuptools import find_packages, setup

package_name = 'transform_tutorial'

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
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_calculator = transform_tutorial.obstacle_calculator:main',
            'turtle_transformer = transform_tutorial.turtle_transformer:main', 
            'turtle_follower = transform_tutorial.turtle_follower:main'

        ],
    },
)
