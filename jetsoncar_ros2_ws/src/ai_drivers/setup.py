from setuptools import setup

package_name = 'ai_drivers'

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
    maintainer_email='equim20@student.jhs.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_loader = ai_drivers.model_loader:main',
            'data_collector = ai_drivers.data_collector:main',
            'sim_bridge = ai_drivers.sim_bridge:main'
        ],
    },
)
