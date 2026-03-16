from setuptools import setup, find_packages
from glob import glob

setup(
    name="g1_io",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/g1_io"]),
        ("share/g1_io", ["package.xml"]),
        ("share/g1_io/launch", glob("launch/*.py")),
        ("share/g1_io/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "isaaclab_adapter = core.adapters.isaaclab.isaaclab_adapter:main",
            "mujoco_adapter   = core.adapters.mujoco.mujoco_adapter:main",
        ],
    },
)
