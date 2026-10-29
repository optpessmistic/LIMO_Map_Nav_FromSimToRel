from setuptools import find_packages, setup

package_name = "limo_maze_perception"
setup(name=package_name, version="0.1.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"])],
      install_requires=["setuptools", "numpy"], zip_safe=True,
      maintainer="LIMO project team", maintainer_email="maintainers@example.com",
      description="Explicitly enabled RGB-D red-box simulation surrogate", license="Apache-2.0",
      entry_points={"console_scripts": ["red_rgbd_detector = limo_maze_perception.node:main"]})
