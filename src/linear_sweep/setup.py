from setuptools import find_packages, setup

package_name = 'linear_sweep'

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
    maintainer='Tuna Gül',
    maintainer_email='tuna.gul@nfrproducts.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "linear_sweep_test = linear_sweep.main:main",
            "nav_bridge = linear_sweep.navigation_bridge:main",
        ],
    },
)
