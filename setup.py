from setuptools import setup, find_packages
packages = find_packages(include=['fourier_grx_dds', 'fourier_grx_dds.*'])
print("Packages found:", packages)
setup(
    name='fourier-grx-dds',
    version='0.1.9_beta',
    packages=find_packages(include=['fourier_grx_dds', 'fourier_grx_dds.*']),
    include_package_data=True,
    package_data={
        'fourier_grx_dds': ['bin/fftai_dds_bridge', 'libraries/*'],
        'fourier_grx_dds.pydds': ['libpydds.so', 'parallel_joints_solver.so'],
    },
    entry_points={
        'console_scripts': [
            'fftai_dds_bridge=fourier_grx_dds.bin.fftai_dds_bridge_wrapper:main',
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
