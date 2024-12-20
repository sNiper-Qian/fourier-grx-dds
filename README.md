# fourier-grx-dds

This is the library for controlling the Fourier GRX robot using DDS communication protocol.

## ❗ Breaking Changes

For users who have been using fourier-grx-client and those who are new to this library, Please note the following features:

- fourier-grx-dds implements the low-level communication protocol using C++, achieving lower resource occupation and faster control frequency.
- Most of the methods in fourier-grx-dds follow the same protocol as in fourier-grx-client, making it easy for fourier-grx-client users to switch over.
## Installation on Ubuntu 20.04
For installation on Ubuntu 22.04, please check out [22.04](https://github.com/sNiper-Qian/fourier-grx-dds/tree/22.04) branch.
#### Install from pypi

Run:

```bash
sudo apt install libtinyxml2-6a
pip install fourier-grx-dds==0.2.8b0
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
The `domain_id` is an identifier used by the bridge to distinguish different robots on the same network. Ensure that `domain_id` is a int number within the range [0, 232] and unique for each robot. The `domain_id` in the configuration file must match the one specified here.

```bash
BRIDGE_CONFIG_FILE=configs/dds/config.gr1t2.json fftai_dds_bridge <domain_id>
```
Run a simple script, which lets robot arms move to a target position and move towards it again using inverse kinematics:
```bash
python examples/run.py --config configs/gr1t2_upper_body.yaml
```

## Usage

Please read the [Tutorial](tutorial.ipynb) for a more detailed step-by-step guide on how to get started and use the interfaces.

Demo scripts can be found in the [examples](examples/) directory.

