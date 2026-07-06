from setuptools import find_packages, setup

package_name = 'qr_nav_controller'

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
    maintainer='zp',
    maintainer_email='zp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 原有的任务管理器入口
            'task_manager = qr_nav_controller.task_manager:main',
            # 新增的总控节点入口
            'all_manager = qr_nav_controller.all_manager:main',
            
            'robot_manager = qr_nav_controller.robot_manager:main',
        ],
    },
)
