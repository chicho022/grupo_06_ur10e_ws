from setuptools import find_packages, setup

package_name = 'grupo06_ur10e_kinematics'

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
    maintainer='Marco Jaldin',
    maintainer_email='marcojaldinmendoza@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fk_node = grupo06_ur10e_kinematics.fk_node:main',
            'ik_node = grupo06_ur10e_kinematics.ik_node:main',
        ],
    },
)
