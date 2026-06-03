from setuptools import find_packages, setup

package_name = "vla_interface_node"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="<TBD>",
    maintainer_email="<TBD>",
    description="Python interface package for Pi0.5 VLA integration and data collection.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
