# This is the required set up file for gym API.
import setuptools
from pathlib import Path

setuptools.setup(
    name='gym_curling',
    version='0.0.1',
    description='A gym environment for curling.',
    long_description=Path("README.md").read_text(),
    long_description_content_type='text/markdown',
    packages=setuptools.find_packages(include="gym_curling*"),
    install_requires=['gym']
)
