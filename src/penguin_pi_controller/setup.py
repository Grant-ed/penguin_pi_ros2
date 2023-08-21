from setuptools import setup

package_name = 'penguin_pi_controller'
submodules = f'{package_name}/penguin_pi_lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grant',
    maintainer_email='grantvanbreda@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'microcontroller_node = penguin_pi_controller.microcontroller_node:main',
            'stress_tester = penguin_pi_controller.stress_test_node:main'
        ],
    },
)
