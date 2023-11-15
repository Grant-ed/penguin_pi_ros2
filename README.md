## Installation / Quick Setup
To install the PenguinPi ROS2 environment, navigate to your home folder and run:
```sh
wget https://raw.githubusercontent.com/Grant-ed/penguin_pi_ros2/main/installation/setup_conda.sh
```
Note: If you encounter a 'connecting, code 443' error, try rerunning the command a few times.
```sh
chmod u+x ./setup_conda.sh
./setup_conda.sh
```
This script will install the mamba environment, clone / build the required repos, and set up a bash alias `a` in your `~/.bashrc` as a shortcut for later commands.

## Running Instructions
Before running ROS2 in a new terminal session, or after rebuilding any packages, always re-source your environment. Use `source install/setup.bash` or simply `a`, which is the alias set during installation. To verify that everything is installed correctly, open a new terminal, source your environment, and run:
```sh
cd ~/penguin_pi_ros2
mamba activate penguinpi_env
source install/setup.bash
ros2 launch diffdrive_penguinpi diffbot.launch.py
```
Alternatively, using the bash alias `a` which was set up in the install script:
```
cd ~/penguin_pi_ros2
a
ros2 launch diffdrive_penguinpi diffbot.launch.py
```
To launch RVIZ on the robot at the same time (not typically recommended due to high resource usage), use the following command:
```
ros2 launch diffdrive_penguinpi diffbot.launch.py rviz:=true
```

## Visualisation Instructions
RViz2 allows you to view the robot's internal model and image output. This can be done on a display connected to the PenguinPi or over the network. In a terminal where the mamba environment is sourced, run:
```sh
rviz2
```
Then, in the RViz2 interface, navigate through File -> Open -> `src/diffdrive_penguinpi/description/rviz/diffbot.rviz`. Alternatively, you can launch RViz2 with the configuration file directly:
```
rviz2 -d src/diffdrive_penguinpi/description/rviz/diffbot.rviz
```
If you want/need run RViz2 on a separate device, make sure to set up the `ROS_DOMAIN_ID` as described in the [networking section](#networking) for proper network communication.

## Keyboard Control
You can control the robot manually using keyboard commands. These commands are interpreted by `ros2_control`, a ROS 2-based framework for controlling hardware. Note: Ensure to open a new terminal and source your environment before executing these commands. The default cmd_vel topic from teleop_keyboard needs to be remapped as follows:
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```

<a id="networking"></a>
## Networking
This section is useful information for using ROS2 across multiple devices (e.g. in order to use visualisation tools).

### Setting the `ROS_DOMAIN_ID`
For ROS 2 networking, set `ROS_DOMAIN_ID` to ensure nodes communicate only within the same domain. Choose any integer between 0 and 255.

**Linux/macOS:**
```sh
export ROS_DOMAIN_ID=10
```
Add to `~/.bashrc` or `~/.zshrc` for permanence.

**Windows:**
```cmd
set ROS_DOMAIN_ID=10
```
For permanent setup, add it to System Environment Variables.

This setting is required on all machines in the network for proper communication, and if it has to be run in each terminal if not set up permanently.
