from setuptools import find_packages, setup

package_name = 'sasv_io'

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
    maintainer='darm',
    maintainer_email='wayanagus.dr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_io_node = sasv_io.dynamixel_io_node:main',
            'px4_io_node = sasv_io.px4_io_node:main',
            'stm_io_node = sasv_io.stm_io_node:main',
        ],
    },
)
