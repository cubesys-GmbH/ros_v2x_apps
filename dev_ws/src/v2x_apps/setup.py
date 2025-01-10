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
    description='Collection of V2X applications utilizing cube devices',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'btp_listener = v2x_apps.btp_listener:main',
                'btp_sender = v2x_apps.btp_sender:main',
                'cam_listener = v2x_apps.cam_listener:main',
                'denm_node = v2x_apps.denm_node:main',
                'cpm_provider = v2x_apps.cpm_provider:main',
                'stationary_vehicle = c2c.stationary_vehicle_trigger:main',
        ],
},
)
