from setuptools import find_packages, setup

package_name = 'yellow_line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yellow_line_follow.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='jijo0718@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_yellow_line = yellow_line_follower.detect_yellow_line:main',
            'yellow_line_center_pub = yellow_line_follower.yellow_line_center_pub:main',
        ],
    },
)
