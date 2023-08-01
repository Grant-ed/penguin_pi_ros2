#!/bin/bash

echo ""
echo "---Installing miniforge3---"
echo ""
sleep 3
cd ~
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh
bash Miniforge3-$(uname)-$(uname -m).sh
source ~/miniforge3/bin/activate
conda config --set auto_activate_base false
conda deactivate
rm -rf Miniforge3-$(uname)-$(uname -m).sh
