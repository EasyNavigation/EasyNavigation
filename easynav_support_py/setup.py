from setuptools import setup

ros_pkg_name = 'easynav_support_py'
py_modules_pkg = 'easynav_goalmanager_py'
setup(
    name=ros_pkg_name,
    version='0.3.2',
    packages=[py_modules_pkg],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + ros_pkg_name]),
        ('share/' + ros_pkg_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco Martín Rico',
    maintainer_email='fmrico@gmail.com',
    description='Support utilities for EasyNav in Python: GoalManagerClient and tests.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goalmanager_demo = easynav_goalmanager_py.demo_client:main',
        ],
    },
)
