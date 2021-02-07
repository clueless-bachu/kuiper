from setuptools import setup

package_name = 'intel_camera'

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
    maintainer='vasista',
    maintainer_email='vasista1997@gmail.com',
    description='A package that contains the sensor suite. It contains programs that will be used to publish all the sensor data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor = intel_camera.sensor:main',
            'view_cam = intel_camera.view_cam:main'
        ],
    },
)
