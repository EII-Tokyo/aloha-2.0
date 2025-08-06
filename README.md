# Instructions

This codebase is forked from the [Interbotix ALOHA repo](https://github.com/Interbotix/aloha), and contains teleoperation and dataset collection and evaluation tools for the Stationary ALOHA kits.

# Quick Start

## Copy the codebase

```bash
cd ~/
git clone https://github.com/EII-Tokyo/aloha -b 2.0
```

## Hardware Guide

## Software Guide

The arm and cameras need to be bound to a unique device. The following sections will provide steps on setting up unique symbolic links for each device.

### Arm Symlink Setup
We will configure udev rules for the arms such that they are bound to the following device names:
- ttyDXL_leader_left
- ttyDXL_leader_right
- ttyDXL_follower_left
- ttyDXL_follower_right

To set these up, do the following:

1. Plug in only the leader left robot to the computer.

2. Determine its device name by checking the `/dev` directory before and after plugging the device in. This is likely something like `/dev/ttyUSB0`.

3. Print out the device serial number by running the following command:
```bash
udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep ATTRS{serial} | head -n 1 | cut -d '"' -f2
```

4. The output of the command will look like `FT88YWBJ` and be the serial number of the arm’s U2D2 serial converter.

5. Add the following line to the computer’s fixed Interbotix udev rules at `/etc/udev/rules.d/99-fixed-interbotix-udev.rules`:

```bash
SUBSYSTEM=="tty", ATTRS{serial}=="<SERIAL NUMBER>", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+="ttyDXL_leader_left"
#                                 ^^^^^^^^^^^^^^^ The result from the previous step
```

6. Repeat for the rest of the arms.

7. To update and refresh the rules, run the following command:

```bash
sudo udevadm control --reload && sudo udevadm trigger
```

8. Plug all arms back into the computer and verify that you can see all devices:

```bash
ls -l /dev/ttyDXL*
```

### Camera Setup

1. Open realsense-viewer

```bash
realsense-viewer
```

**Note**

If realsense-viewer is not already installed on your machine, follow these steps on the [librealsense GitHub repository](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) to install librealsense2-utils.

2. Plug in a single camera and check the sidebar for its entry. If it does not show up in the side bar, click Add Source and find the Intel RealSense D405 in the drop down.

3. Click on Info for the camera, find the Serial Number, and copy it.
![Serial Number of Camera](./images/rsviewer_serialno2.png)

4. Put the camera serial number in the appropriate config entry at `~/aloha/config/robot/aloha_stationary.yaml`.

5. Repeat for the rest of the cameras. If the workspace has not been symbolically-linked, a rebuild may be necessary.

## Operation Commands

- Start docker and bring up the robot
```bash
docker run --rm -it --memory=16g --network=host -v /dev:/dev -v .:/root/interbotix_ws/src/aloha --privileged lyl472324464/robot:aloha-stationary
ros2 launch aloha aloha_bringup.launch.py robot:=aloha_stationary # launch hardware drivers and control software
```

- Shut down
```bash
docker ps # get the container id
docker exec -it <container_id> /bin/bash # enter the container
export INTERBOTIX_ALOHA_IS_MOBILE=false # true for Mobile, false for Stationary
cd /root/interbotix_ws/src/aloha/scripts/
python3 sleep.py -r aloha_stationary -a
```

- Teleop
```bash
docker ps # get the container id
docker exec -it <container_id> /bin/bash # enter the container
cd /root/interbotix_ws/src/aloha/scripts/
python3 teleop.py -r aloha_stationary -t
```

- Record episodes
```bash
docker ps # get the container id
docker exec -it <container_id> /bin/bash # enter the container
cd /root/interbotix_ws/src/aloha/scripts/
python3 record_episodes.py \
      --task_name aloha_stationary \
      --robot aloha_stationary \
      [--episode_idx <episode_idx>] \
      [-b, --enable_base_torque] \
      [-g, --gravity_compensation]
      
python3 record_episodes.py --task_name aloha_stationary --robot aloha_stationary
```


- Visualize episodes 
```bash
docker ps # get the container id
docker exec -it <container_id> /bin/bash # enter the container
cd /root/interbotix_ws/src/aloha/scripts/
python3 visualize_episodes.py --dataset_dir ../aloha_data/aloha_stationary/ --episode_idx 7 -r aloha_stationary
```

- Replay episodes
```bash
docker ps # get the container id
docker exec -it <container_id> /bin/bash # enter the container
cd /root/interbotix_ws/src/aloha/scripts/
python3 replay_episodes.py --dataset_dir ../aloha_data/aloha_stationary/ --episode_idx 7 -r aloha_stationary
```

# Structure
- [``aloha``](./aloha/): Python package providing useful classes and constants for teleoperation and dataset collection.
- [``config``](./config/): a config for each robot, designating the port they should bind to, more details in quick start guide.
- [``launch``](./launch): a ROS 2 launch file for all cameras and manipulators.
- [``scripts``](./scripts/): Python scripts for teleop and data collection