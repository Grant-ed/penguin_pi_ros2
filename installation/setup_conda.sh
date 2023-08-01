#!/bin/bash

cd ~

## Pre-reqs
echo ""
echo "---Installing pre-reqs---"
echo ""
sleep 3
sudo apt update
sudo apt upgrade -y
sudo apt install -y git python3-pip pre-commit mesa-utils

## Make workspace
cd ~/

## Clone PenguinPi repo
echo ""
echo "---Cloning PenguinPi repo---"
echo ""
sleep 3
git clone --recurse-submodules https://github.com/Grant-ed/penguin_pi_urdf.git

## Download and install mambaforge
source ~/penguin_pi_urdf/installation/install_scripts/install_mambaforge.sh

## Create ros_env development environment
source ~/penguin_pi_urdf/installation/install_scripts/install_conda_env.sh

## Create an alias for ease
echo "alias a='mamba activate penguinpi_env && source install/setup.bash'" >> ~/.bashrc

echo ""
echo "---Building packages---"
echo ""
sleep 3
colcon build --symlink-install
source ~/.bashrc
a

## Wrap up
echo "Thank you for installing"
