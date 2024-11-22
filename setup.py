from setuptools import setup, find_packages
packages = find_packages(include=['robot_control', 'robot_control.*'])
print("Packages found:", packages)
setup(
    name='fourier_utils',
    version='0.1.1_beta',
    packages=find_packages(include=['robot_control', 'robot_control.*']),
    include_package_data=True,
    package_data={
        'robot_control': ['bin/fftai_dds_bridge', 'libraries/*.so'],
        'robot_control.pydds': ['libpydds.so', 'parallel_joints_solver.so'],
    },
    entry_points={
        'console_scripts': [
            'fftai_dds_bridge=robot_control.bin.fftai_dds_bridge_wrapper:main',
        ],
    },
    zip_safe=False,
    classifiers=[
        'Programming Language :: Python :: 3',
    ],
    install_requires=[
        'numpy==1.26.4',
        'omegaconf==2.3.0',
        'pin-pink==3.0.0',
        'meshcat==0.3.2',
        'loguru==0.7.2'
    ],
    python_requires='>=3.11',
)
