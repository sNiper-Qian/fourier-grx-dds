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
pip install fourier-grx-dds
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

## Quick Start
Start the DDS bridge:
```bash
BRIDGE_CONFIG_FILE=configs/dds/config.gr1.json fftai_dds_bridge
```
Run a simple script, which lets robot arms move to a target position and move towards it again using inverse kinematics:
```bash
python examples/run.py --config configs/gr1_upper_body.yaml
```

## Usage

Please read the [Tutorial](tutorial.ipynb) for a more detailed step-by-step guide on how to get started and use the interfaces.

Demo scripts can be found in the [examples](examples/) directory.

