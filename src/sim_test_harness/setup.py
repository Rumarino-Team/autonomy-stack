from setuptools import find_packages, setup

package_name = 'sim_test_harness'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joseburgosguntin',
    maintainer_email='101651981+joseburgosguntin@users.noreply.github.com',
    description='Volumetric trigger testing framework for headless simulation CI/CD',
    license='MIT',
    entry_points={
        'console_scripts': [
            'volume_trigger_node = sim_test_harness.volume_trigger_node:main',
        ],
    },
)
