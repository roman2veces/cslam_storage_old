from setuptools import setup

package_name = 'cslam_storage'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='romantwice',
    maintainer_email='romanroman2705@gmail.com',
    description='This package allow us to store the map as pcd and json files in the robots or in the base station',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'cslam_storage = cslam_storage.cslam_storage:main',
        ],
    },
)
