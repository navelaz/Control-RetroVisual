from setuptools import find_packages, setup

package_name = 'sphere_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/move_follow.launch.py']),  # Corregido
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='navelaz',
    maintainer_email='navelaz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_sphere = sphere_follower.move_sphere:main',
            'follow_sphere = sphere_follower.follow_sphere:main',
        ],
    },
)
