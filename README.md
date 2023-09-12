# lsdo_project_template

<!---
[![Python](https://img.shields.io/pypi/pyversions/lsdo_project_template)](https://img.shields.io/pypi/pyversions/lsdo_project_template)
[![Pypi](https://img.shields.io/pypi/v/lsdo_project_template)](https://pypi.org/project/lsdo_project_template/)
[![Coveralls Badge][13]][14]
[![PyPI version][10]][11]
[![PyPI Monthly Downloads][12]][11]
-->

[![GitHub Actions Test Badge](https://github.com/LSDOlab/lsdo_project_template/actions/workflows/actions.yml/badge.svg)](https://github.com/lsdo_project_template/lsdo_project_template/actions)
[![Forks](https://img.shields.io/github/forks/LSDOlab/lsdo_project_template.svg)](https://github.com/LSDOlab/lsdo_project_template/network)
[![Issues](https://img.shields.io/github/issues/LSDOlab/lsdo_project_template.svg)](https://github.com/LSDOlab/lsdo_project_template/issues)

# Instructions for Setting Up Your Environment

## Update and Install System Packages

```bash
sudo apt-get update
sudo apt-get dist-upgrade
sudo apt-get install build-essential
sudo apt-get install ffmpeg libsm6 libxext6
```

## Create and Set Up Conda Environment
Create a new Conda environment named 'caddee' with Anaconda and Python 3.10:
```bash
conda create -n caddee anaconda python=3.10
```

Once the environment is set up, activate it and install the required Python packages:
```bash
conda activate caddee
pip install smt vedo
```

Within the caddee environment, you'll also need to clone and install several repositories. Use the ```pip3 install -e .``` command within each repository's directory after cloning.

## Troubleshooting
If you encounter any issues when plotting the mesh, consult [this link](https://github.com/conda-forge/ctng-compilers-feedstock/issues/95#issuecomment-1449848343) for potential solutions.


# License
This project is licensed under the terms of the **GNU Lesser General Public License v3.0**.
