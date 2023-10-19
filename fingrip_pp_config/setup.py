from setuptools import setup, find_packages
import os
from glob import glob

main_package = "fingrip_pp_config"

setup(
    name=main_package,
    version="0.0.0",
    packages=find_packages(include=[main_package]),
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ("share/ament_index/resource_index/packages", ["resource/" + main_package]),
        # Include our package.xml file
        ("share/" + main_package, ["package.xml"]),
        # Include all launch files.
        (
            os.path.join("share", main_package, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        # Include all config files
        (os.path.join("share", main_package, "config"), glob("*.yaml")),
    ],
    # Dependencies needed to use package
    install_requires=["setuptools"],
    # Dependencies required for testing
    # setup_requires=['pytest-runner'],
    tests_require=["pytest"],
    # Running python setup.py pytest will execute all tests stored in
    #  the tests folder
    zip_safe=True,
    maintainer="alexis",
    maintainer_email="alexis-h-34@hotmail.fr",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        # nom_executable = nom_folder.nomfufichiersource:main
        "console_scripts": [
            "simulate_node = fingrip_pp_config.simulate:main",
        ],
    },
)
