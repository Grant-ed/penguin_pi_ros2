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

## Limitations

This section outlines the current limitations of the PenguinPi ROS2 environment and possible directions for future development. It's intended to guide future developers in understanding and potentially resolving these constraints.

### Exclusive Access to Serial Port by ROS2 Control

- **Issue:** Currently, the serial port used for communication with the microcontroller can only support a single connection. The `ros2_control` initializes the instance of `penguinpi_comms`, thereby gaining exclusive access to the serial port. This exclusivity prevents other nodes from setting the LCD screen text or reading buttons.
- **Possible Solution:** A potential fix involves the use of a shared or global instance of `penguinpi_comms`. This adjustment could allow multiple nodes to interact with the serial port concurrently.

### Handling Receiving of Unrequested Microcontroller Text Messages

- **Issue:** The current implementation does not adequately handle text received from the microcontroller. This oversight leads to the `ros2_control` process crashing when the power supply is disconnected, as the microcontroller sends a message to the Pi that is not processed.
- **Suggested Improvement:** Implement a separate read thread for handling microcontroller communications, similar to the previous Python implementation in `src/ppi_controller/ppi_controller/penguin_pi_lib/penguinPi.py -> UART -> uart_recv`. This would improve system robustness, especially during power supply disruptions, or when the .

### Dependency on Libserial Repository Location

- **Issue:** The current setup requires the `libserial` repository to be cloned into the user's home directory. This requirement may not be ideal in all environments and could pose a limitation in terms of flexibility and system organization.
- **Recommendation:** Future development could focus on making the system more flexible regarding the location of the `libserial` repository, potentially allowing it to reside in different directories based on user preference or system requirements.

### Alternative Approaches to Serial Communication

- **Observation:** Other solutions for serial communication, such as the ROS2 Serial Bridge, have been explored. However, these packages primarily work with publishers and subscribers, which are not compatible with `ros2_control`'s intended lower-level operations.
- **Consideration:** While integrating such packages may be technically feasible, it is not advised due to the conceptual mismatch with `ros2_control`. Future research could explore more compatible methods of serial communication that align with the low-level nature of `ros2_control`.
 
