from setuptools import setup

package_name = 'penguin_pi_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='grantvanbreda@gmail.com',
    maintainer='Your Name',
    maintainer_email='grantvanbreda@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'microcontroller_node = penguin_pi_urdf.microcontroller_node:main',  # Update with the correct path to your node's main function
        ],
    },
)
