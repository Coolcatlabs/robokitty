import setuptools

DIST_NAME = "robokitty"
DESCRIPTION = "A package for controlling and managing multiped robotic systems."
PYTHON_REQUIRES = ">=3.12"

REQUIRES = ["pyserial==3.*"]

DEV_REQUIRES = [
    "pre-commit==4.5.*",
]


with open("README.md") as f:
    LONG_DESCRIPTION = f.read()


def setup_package():
    metadata = dict(
        name=DIST_NAME,
        description=DESCRIPTION,
        long_description=LONG_DESCRIPTION,
        packages=setuptools.find_packages(),
        python_requires=PYTHON_REQUIRES,
        install_requires=REQUIRES,
        extras_require={"dev": DEV_REQUIRES},
        entry_points={
            "console_scripts": [
                "robokitty = robokitty.__main__:main",
            ],
        },
    )

    setuptools.setup(**metadata)


if __name__ == "__main__":
    setup_package()
