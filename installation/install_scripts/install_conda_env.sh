#!/bin/bash

echo ""
echo "---Creating penguinpi env---"
echo ""
sleep 3
cd ~/penguin_pi_urdf
mamba env create --name penguinpi_env --file installation/humble_py310_dev_env.yml
conda activate penguinpi_env
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
mamba install -y --file installation/conda_requirements.txt
