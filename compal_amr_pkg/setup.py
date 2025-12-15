from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'compal_amr_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This installs compal_amr.yaml to the share directory
        (os.path.join('share', package_name), glob('compal_amr.yaml')),
    ],
    # All necessary dependencies
    install_requires=['setuptools', 'numpy', 'pyyaml', 'rclpy', 'geometry_msgs', 'std_msgs', 'ament_index_python'],
    zip_safe=True,
    maintainer='csl',
    maintainer_email='10335111tw@gmail.com',
    description='ROS 2 package for AMR command velocity to motor command kinematics conversion.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Correct entry point for running the node
            'cmd_vel_kinematics_node = compal_amr_pkg.compal_amr_controller:main',
        ],
    },
)