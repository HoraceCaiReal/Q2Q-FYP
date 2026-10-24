from setuptools import setup

package_name = 'q2q_low'

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
    description='下层执行器：抓取流水线 action 适配',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'grasp_executor = q2q_low.grasp_executor:main',
        'suction_relay = q2q_low.suction_relay:main',
    ]},
)