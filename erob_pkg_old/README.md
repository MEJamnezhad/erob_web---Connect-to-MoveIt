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