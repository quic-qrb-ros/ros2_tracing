from setuptools import find_packages
from setuptools import setup

package_name = 'tracetools_trace'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='Christophe Bedard',
    maintainer_email='fixed-term.christophe.bourquebedard@de.bosch.com',
    author='Christophe Bedard',
    author_email='fixed-term.christophe.bourquebedard@de.bosch.com',
    # url='',
    keywords=['ROS'],
    description='Tools for setting up tracing sessions',
    entry_points={
        'console_scripts': [
            f'trace = {package_name}.trace:main',
        ],
    },
    license='TODO',
    tests_require=['pytest'],
)
