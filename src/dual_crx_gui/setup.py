from setuptools import find_packages, setup

package_name = "dual_crx_gui"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Howard",
    maintainer_email="howard@example.com",
    description="Dual CRX standalone GUI client",
    license="Apache-2.0",
    entry_points={"console_scripts": ["dual_crx_gui = dual_crx_gui.app:main"]},
)
