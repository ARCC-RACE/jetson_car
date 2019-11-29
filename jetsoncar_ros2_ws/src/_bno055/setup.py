from setuptools import setup

package_name = 'b_no055'

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
 maintainer='michael',
 maintainer_email='michael@todo.todo',
 description='The bno055 package',
 license='TODO',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'bno055_ros = src.bno055_ros:main'
     ],
   },
)