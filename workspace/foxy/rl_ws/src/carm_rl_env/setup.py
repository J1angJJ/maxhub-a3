from setuptools import setup

package_name = "carm_rl_env"

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
        "stable-baselines3==2.3.2",
    ],
    zip_safe=True,
    maintainer="j1angjj",
    maintainer_email="j1angjj@todo.invalid",
    description="Minimal Gymnasium-style reinforcement learning environments for CArm / MAXHUB A3.",
    license="UNLICENSED",
    entry_points={
        "console_scripts": [
            "random_rollout = carm_rl_env.random_rollout:main",
            "train_reaching = carm_rl_env.train_reaching:main",
            "evaluate_reaching = carm_rl_env.evaluate_reaching:main",
            "trace_reaching = carm_rl_env.trace_reaching:main",
        ],
    },
)
