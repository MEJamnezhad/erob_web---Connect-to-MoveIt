`eRob node` uses [pysoem](https://github.com/bnjmnp/pysoem/tree/master) to communicate to the ZeroErr arm through EtherCAT. The **pysoem** module requires sudo/admin privileges to access the network interface (**eno1**).

# Ways to run:
## `ros2 launch`
 - cd into workspace `cd /home/arol6/arm2_ws/`
 - source workspace `source ./install/setup/bash`
 - launch the package `ros2 launch erob_pkg test.launch.py`

## `sudo su`
 - Enter `sudo su` into a terminal
 - cd into workspace `cd /home/arol6/arm2_ws/`
 - source workspace `source ./install/setup/bash`
 - run the node `ros2 run erob_pkg erob_node`

# Notes
- The file */etc/sudoers* was modified to allow current user (**arol6**) to invoke `sudo` without password prompt
  - To modify/read, enter into terminal `sudo visudo`







# The following command will disable GUI on boot hence upon the reboot the system will boot into multi user target:
  - `$ sudo systemctl set-default multi-user`

## Reboot or log out from a current session to exit GUI:
- `$ reboot`
    OR
- `$ gnome-session-quit`

## How to enable GUI to start on boot
Given that you have installed GUI on your Ubuntu 22.04 Server/Desktop you can enable the system to start to GUI by execution of the following command:
- `$ sudo systemctl set-default graphical`

## Start GUI manually from a command line
For a GNOME GUI installations using GDM as a default display manager you can start GUI from a command line by executing the below command:
- `$ sudo systemctl start gdm3`


# Multi Terminals
Use alt+ctrl+[F1-F12] to open different terminal






 