from setuptools import setup, find_packages

package_name = 'rclpy_components'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + 'test_composition'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhen Ju',
    maintainer_email='juzhen@huawei.com',
    description='The dynamic node management package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'component_container = rclpy_components.component_container:main',
            'component_container_mt = rclpy_components.component_container_mt:main',
        ],
        'rclpy_components': [
            'test_composition::TestFoo = rclpy_components_test:TestFoo',
        ]
    },
)
