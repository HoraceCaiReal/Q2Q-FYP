from setuptools import setup

package_name = 'q2q_orchestration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cai Haoxian',
    maintainer_email='q2q@fyp.local',
    description='中层调度与互锁：第一片只读世界状态',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'orchestration_node = q2q_orchestration.orchestration_node:main',
        ],
    },
)
