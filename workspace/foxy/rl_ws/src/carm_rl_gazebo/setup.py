from setuptools import setup

package_name = "carm_rl_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "gymnasium==0.29.1",
        "numpy>=1.23,<2",
    ],
    zip_safe=True,
    maintainer="j1angjj",
    maintainer_email="j1angjj@todo.invalid",
    description="Gymnasium-style Gazebo reaching environment for CArm / MAXHUB A3.",
    license="UNLICENSED",
    entry_points={
        "console_scripts": [
            "random_gazebo_rollout = carm_rl_gazebo.random_gazebo_rollout:main",
            "train_gazebo_reaching = carm_rl_gazebo.train_gazebo_reaching:main",
            "evaluate_gazebo_reaching = carm_rl_gazebo.evaluate_gazebo_reaching:main",
            "trace_gazebo_reaching = carm_rl_gazebo.trace_gazebo_reaching:main",
        ],
    },
)
