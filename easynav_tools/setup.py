from setuptools import find_packages, setup

package_name = 'easynav_tools'
setup(
    name=package_name,
    version='0.3.2',
    packages=find_packages(
        include=[package_name, package_name + '.*'], exclude=['test', 'scripts']
    ),
    include_package_data=True,
    package_data={
        # include everything under easynav_tools/vendor in the installed package
        'easynav_tools': ['vendor/**'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'rich>=13.3.0', 'pydantic>=2.0.0'],
    zip_safe=False,
    maintainer='Francisco Martín Rico',
    maintainer_email='fmrico@gmail.com',
    description='ROS 2 Navigation tools: TUI (Textual) + ros2cli commands for EasyNav.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tui = easynav_tools.tui.app:run_app',
        ],
        # ros2 easynav <verb>
        'ros2cli.command': [
            'easynav = easynav_tools.cli.easynav:EasynavCommand',
        ],
        # IMPORTANT: extension point + verb group for ros2cli
        'ros2cli.extension_point': [
            'easynav.verb = ros2cli.verb:VerbExtension',
        ],
        'easynav.verb': [
            'navigation_control = easynav_tools.cli.navigation_control:NavigationControlVerb',
            'goal_info = easynav_tools.cli.goal_info:GoalInfoVerb',
            'twist = easynav_tools.cli.twist:TwistVerb',
            'nav_state = easynav_tools.cli.nav_state:NavStateVerb',
            'timestats = easynav_tools.cli.timetats:TimeStatsVerb',
            'plugins = easynav_tools.cli.plugins:PluginsVerb',
        ],
    },
)
