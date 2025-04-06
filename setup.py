from setuptools import find_packages, setup

package_name = 'frontier'

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
    maintainer='paterson',
    maintainer_email='patersonwong@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav = github_final_nav.frontier_nav:main',
            'frontier = github_final_nav.test_navigate:main',
            #'new_nav = frontier.frontier_new:main'
        ],
    },
)
