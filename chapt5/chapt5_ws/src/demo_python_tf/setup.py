from setuptools import find_packages, setup

package_name = 'demo_python_tf'

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
    maintainer='ubuntu',
    maintainer_email='1252759359@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'static_tf_broadcast = demo_python_tf.static_tf_broadcast:main',
            'tf_broadcast = demo_python_tf.tf_broadcast:main',
            'tf_listener = demo_python_tf.tf_listener:main',
        ],
    },
)
