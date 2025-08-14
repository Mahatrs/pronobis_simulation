from setuptools import find_packages, setup

package_name = 'med_joint_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maha',
    maintainer_email='maha@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotation_publisher = med_joint_controller.rotation_publisher:main',
            'translation_publisher = med_joint_controller.translation_publisher:main',
            'move_to_pick_pose = med_joint_controller.move_to_pick_pose:main',
            'smooth_translation_pub = med_joint_controller.smooth_translation_pub:main',
            'rotation_x_publisher = med_joint_controller.rotation_x_publisher:main',
            'rotation_y_publisher = med_joint_controller.rotation_y_publisher:main',
            'translation_x_publisher = med_joint_controller.translation_x_publisher:main',
            'translation_y_publisher = med_joint_controller.translation_y_publisher:main',
            'Randomcmd= med_joint_controller.Randomcmd:main',
            'VerticalPub= med_joint_controller.VerticalPub:main',

        ],  
    },
)
