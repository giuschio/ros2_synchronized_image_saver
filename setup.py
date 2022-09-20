from setuptools import setup

package_name = 'synchronized_image_saver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Giulio Schiavi',
    author_email='giuschio98@gmail.com',
    maintainer='Giulio Schiavi',
    maintainer_email='giuschio98@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Apply a TimeSynchronizer to two image streams and save the images when timestamps match.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node ='' synchronized_image_saver.node:main',
        ],
    },
)
