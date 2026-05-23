from setuptools import setup

package_name = 'voice_nav_pkg'

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
    maintainer='seoyeong',
    maintainer_email='seoyeong@todo.todo',
    description='Medical Robot Voice Nav',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_goal_publisher = voice_nav_pkg.voice_goal_publisher:main',
            'voice_text_publisher = voice_nav_pkg.voice_text_publisher:main', 
        ],
    },
)
