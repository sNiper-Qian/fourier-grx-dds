# fourier-grx-dds

This is the library for controlling the Fourier GRX robot using DDS communication protocol.

## ❗ Breaking Changes

For users who have been using fourier-grx-client and those who are new to this library, Please note the following features:

- fourier-grx-dds implements the low-level communication protocol using C++, achieving lower resource occupation and faster control frequency.
- Most of the methods in fourier-grx-dds follow the same protocol as in Fourier-GRX-Client, making it easy for Fourier-GRX-Client users to switch over.
## Installation

#### Install from pypi

Run:

```bash
sudo apt install libtinyxml2-6a
pip install fourier-grx-dds==0.1.2b0

```

#### Install from source

```bash
git clone https://gitee.com/FourierIntelligence/fourier-grx-dds.git
cd fourier-grx-dds
pip install -e .
```
#### Configure environment
This script will add the necessary LD_LIBRARY_PATH to .bashrc

```bash
source configure.sh
source ~/.bashrc
```


## Usage

Please read the [Tutorial](tutorial.ipynb) for a step-by-step guide on how to get started and use the interfaces.

Demo scripts can be found in the [examples](examples/) directory.
