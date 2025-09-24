from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'jobot_vad_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model'), glob('model/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zq',
    maintainer_email='zhaoqi.chen@spacemit.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = jobot_vad_py.my_node:main',
            'vad_node = jobot_vad_py.vad_node:main'
        ],
    },
)
