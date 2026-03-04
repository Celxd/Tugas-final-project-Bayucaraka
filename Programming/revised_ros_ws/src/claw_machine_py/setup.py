from setuptools import find_packages, setup

package_name = 'claw_machine_py'

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
    maintainer='',
    maintainer_email='',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    package_data={
        'claw_machine_py': ['calib.npz'],
    },
    entry_points={
        'console_scripts': [
            'apriltag_node = claw_machine_py.apriltag_node:main',
        ],
    },
)
