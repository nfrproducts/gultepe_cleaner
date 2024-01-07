from setuptools import find_packages, setup

package_name = 'gultepe_scripts'

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
    maintainer='tunaprogrammer',
    maintainer_email='tuna.gul@nfrproducts.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "grid_sweeper = gultepe_scripts.grid_sweeper:main",
            "nav_cmd_test = gultepe_scripts.nav_bridge:test",
        ],
    },
)
