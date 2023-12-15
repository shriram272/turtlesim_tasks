from setuptools import setup

package_name = 'cv_basics'

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
    maintainer='shriram',
    maintainer_email='dshriram27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "webcam=cv_basics.webcam:main",
            "webcam2=cv_basics.webcam2:main",
            "turt=cv_basics.turt:main" ,
            "decel=cv_basics.decel:main",
            "circle=cv_basics.circle:main",
            "police=cv_basics.police:main",
            "police2=cv_basics.police2:main",
            "police3=cv_basics.police3:main"
        ],
    },
)
