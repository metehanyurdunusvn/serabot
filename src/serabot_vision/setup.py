from setuptools import find_packages, setup

package_name = 'serabot_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samet760',
    maintainer_email='samedgencc@gmail.com',
    description='Vision-guided red plant approach with LiDAR safety',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'red_plant_tracker = serabot_vision.red_plant_tracker:main',
    ]},
)
