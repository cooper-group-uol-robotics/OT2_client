from setuptools import setup, find_packages

setup(
    name="OT2_client_package",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[],
    description="A client package to bridge ROS and OT2 module",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    author="Satheesh",
    author_email="sathiz.v@gmail.com",
    url="https://github.com/yourusername/my_package",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
