# Simple Grasper

ROS node for sending grasping pose and hand configurations using MoveIt! with
KUKA and BHand.

## Install

For Ubuntu 16.04, ROS Kinetic.

Within the catkin workspace src directory:

```bash
git clone https://github.com/auth-arl/simple_grasper.git
git clone https://github.com/auth-arl/autharl_core.git
cd autharl_core
git checkout simple-grasper-devel
./install.sh
```

Then `catkin_make`.

## Run

Run the following launch files/nodes in different terminals:

Before anything launch RViz and URDF:

```bash
roslaunch rosba_launch display.launch
```

Run grasper which provides `home` and `grasp` services:

```bash
roslaunch rosba_grasper run.launch
```

Camera:

```bash
roslaunch openni2_launch openni2.launch camera:=asus_xtion publish_tf:=false
```

If you want to test if BHand works run:

```bash
rosrun bhand_test bhand_robot_test
```

If BHand does not initialized properly (Finger 1 might not open) initialize multiple times with:

```bash
rosrun bhand_test bhand_reset
```

## Services

- `home`: Sends the KUKA arm to home position.
- `grasp`: Sends the KUKA arm to grasping pose and closes the fingers to the desired configuration.

## Configuration

Set parameters in `rosba_grasper/config/config.yml`.
