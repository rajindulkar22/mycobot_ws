import os

from setuptools import find_packages, setup

package_name = "mycobot_yolo_assets"


def collect_asset_files(source_dir, install_prefix):
    data_files = []
    for root, _, files in os.walk(source_dir):
        if not files:
            continue
        rel_root = os.path.relpath(root, source_dir)
        if rel_root == ".":
            dest = install_prefix
        else:
            dest = os.path.join(install_prefix, rel_root)
        sources = [os.path.join(root, name) for name in files]
        data_files.append((dest, sources))
    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ]
    + collect_asset_files(
        "assets",
        os.path.join("share", package_name, "assets"),
    ),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="YOLO dataset, weights, and base model for cube detection.",
    license="TODO: License declaration",
)
