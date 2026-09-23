from setuptools import find_packages, setup

package_name = 'robomaster_ep_demos'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pick_place.launch.py']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='RoboMaster simulation maintainer', maintainer_email='maintainer@example.com',
    description='Repeatable deterministic pick-place demo', license='MIT',
    entry_points={'console_scripts': [
        'pick_place = robomaster_ep_demos.pick_place:main',
        'verify_simulation = robomaster_ep_demos.verify_simulation:main',
    ]},
)
