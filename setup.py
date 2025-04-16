from setuptools import setup

package_name = 'rosbag2_to_video'

setup(
    name=package_name,
    version='1.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PySide6', 'qt-material'],
    zip_safe=True,
    author='Ivan Santiago Paunovic',
    author_email='ivanpauno@ekumenlabs.com',
    maintainer='Błażej Sowa',
    maintainer_email='blazej@ficionlab.pl',
    url='https://github.com/ros2/rosbag2_to_video',
    download_url='https://github.com/ros2/rosbag2_to_video/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Exports ROS2 bag files to video.',
    long_description="Provides a node that can export ROS2 bag files to video.",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosbag2_to_video = rosbag2_to_video:main',
            'rosbag2_to_video_gui = rosbag2_to_video.gui:main',
        ],
        'ros2bag.verb': [
            'to_video = rosbag2_to_video.ros2bag_verb:Rosbag2ToVideoVerb',
        ],
    },
)
