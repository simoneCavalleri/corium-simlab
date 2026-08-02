import os
from setuptools import setup, find_packages

setup(
    name="corium-sim",
    version="0.2.0",
    description="High-Performance 3D Agent Simulation & WebGPU Engine",
    long_description=open("README.md").read() if os.path.exists("README.md") else "",
    long_description_content_type="text/markdown",
    author="Simone Cavalleri",
    license="MIT",
    package_dir={"": "python"},
    packages=find_packages(where="python"),
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "corium-sim=corium_sim.cli:main",
        ],
    },
)
