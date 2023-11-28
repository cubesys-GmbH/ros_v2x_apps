from setuptools import find_packages, setup

package_name = 'v2x_apps'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Apostolos Georgiadis',
    maintainer_email='apostolos.georgiadis@nfiniity.com',
    description='Sample V2X Application',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'cam_listener = v2x_apps.cam_listener:main',
        ],
},
)