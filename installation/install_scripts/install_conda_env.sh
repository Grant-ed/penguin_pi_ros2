#!/bin/bash

echo ""
echo "---Creating penguinpi env---"
echo ""
sleep 3
cd ~/penguin_pi_urdf

mamba init
eval "$(conda shell.bash hook)"
source ~/miniforge3/etc/profile.d/mamba.sh

# https://robostack.github.io/GettingStarted.html
mamba env create --name penguinpi_env --file installation/humble_py310_dev_env.yml
mamba activate penguinpi_env
# this adds the conda-forge channel to the new created environment configuration 
conda config --env --add channels conda-forge
# and the robostack channel
conda config --env --add channels robostack-staging
# remove the defaults channel just in case, this might return an error if it is not in the list which is ok
conda config --env --remove channels defaults

mamba install -y --file installation/conda_requirements.txt
