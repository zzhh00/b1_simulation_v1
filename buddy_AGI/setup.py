from setuptools import setup

package_name = 'buddy_AGI'

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
    maintainer='susan',
    maintainer_email='suzhenxzhong@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python', 
        'Programming Language :: Python :: 3.8', 
    ],
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # add your scripts here in the format:
            # 'script_name = package_name.folder_name.file_name:main'
            # for example if your script is located in
            # `src/buddy_AGI/scripts/buddy_demo_agi.py` and has a `main` function:
            # 'buddy_demo_agi = buddy_AGI.scripts.buddy_demo_agi:main',
        ],
    },
)
