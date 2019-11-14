# Simple Grasper

ROS node for sending grasping pose and hand configurations using MoveIt! with
KUKA and BHand.

## Install

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

```bash
roslaunch rosba_launch display.launch
```

In another terminal:

```bash
roslaunch rosba_grasper run.launch
```

## Services

- `home`: Sends the KUKA arm to home position.
- `grasp`: Sends the KUKA arm to grasping pose and closes the fingers to the desired configuration.

## Configuration

Set parameters in `rosba_grasper/config/config.yml`.
