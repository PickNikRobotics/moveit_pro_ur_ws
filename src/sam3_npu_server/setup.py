from glob import glob

from setuptools import find_packages, setup

package_name = "sam3_npu_server"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
        ("share/" + package_name + "/assets", glob("assets/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="MoveIt Pro User",
    maintainer_email="support@picknik.ai",
    description="SAM3 on the IQ10 NPU, served to GetSam3Masks from a root sidecar container.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["sam3_npu_server = sam3_npu_server.server:main"]},
)
