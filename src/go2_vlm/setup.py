from glob import glob

from setuptools import setup

package_name = 'go2_vlm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riderman10000',
    maintainer_email='rhimeshlwagun1@gmail.com',
    description='Vision-language model -> Nav2 keepout mask and speed limit',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_costmap = go2_vlm.semantic_costmap:main',
        ],
    },
)
