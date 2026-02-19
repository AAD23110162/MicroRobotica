from setuptools import find_packages, setup

package_name = 'conv_pkg'

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
    maintainer='codespace',
    maintainer_email='159966137+AAD23110162@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
   entry_points={
        'console_scripts': [
            'talker_f = conv_pkg.pub_fahrenheit:main',
            'converter_c = conv_pkg.conv_celsius:main',
        ],
    },
)
