from setuptools import find_packages, setup

package_name = 'my_bug0'

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
    maintainer='yuki',
    maintainer_email='pattharajarin.p@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
        entry_points={
        'console_scripts': [
            'bug0_navigator = my_bug0.bug0_navigator:main',
        ],
    },
)
