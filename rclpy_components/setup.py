from setuptools import find_packages, setup

package_name = 'rclpy_components'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Pande, Shane Loretz',
    maintainer_email='aditya.pande@openrobotics.org, sloretz@openrobotics.org',
    description='The dynamic node management package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'component_container = rclpy_components.component_container:main',
            'component_container_mt = rclpy_components.component_container_mt:main',
        ],
        'rclpy_components': [
            'rclpy_components::FooNode = rclpy_components.test.foo_node:FooNode',
        ]
    },
)
