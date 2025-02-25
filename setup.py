from setuptools import setup, find_packages

setup(
    name="motion",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "pykos",
    ],
    author="Verda Korzeniewski",
    author_email="verda@example.com",  # Replace with your actual email
    description="Robot motion control library",
    keywords="robotics, motion control",
    python_requires=">=3.8",
)