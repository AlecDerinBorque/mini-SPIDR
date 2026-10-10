from setuptools import find_packages, setup

package_name = 'mini_spidr'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alec',
    maintainer_email='AlecDerinBorque@users.noreply.github.com',
    description='Mini-SPIDR: simulated pan/tilt tracking pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'frame_node = mini_spidr.frame_node:main',
            'detector_node = mini_spidr.detector_node:main',
            'ptu_sim_node = mini_spidr.ptu_sim_node:main',
            'controller_node = mini_spidr.controller_node:main',
        ],
    },
)
