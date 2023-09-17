# sawSensablePhantomROS2

ROS 2 node for sawSensablePhantom (https://github.com/jhu-saw/sawSensablePhantom).

## Compilation

First install dependencies for cisst/SAW:
```sh
sudo apt install python3-vcstool python3-colcon-common-extensions libxml2-dev libraw1394-dev libncurses5-dev libncurses5 qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev
```

You can use the ROS 2 VCS python-based tool (`sudo apt install python3-vcstool`) to download all the cisst, sawSensablePhantom and ROS 2 specific packages needed for the sawSensablePhantom ROS 2 node.  Look for the `.repos` file in this repository and find the link to the raw content.  Then in your ROS 2 workspace, under the `src` directory, use:
```sh
vcs import --input https://raw.githubusercontent.com/jhu-saw/sawSensablePhantomROS2/main/sensable_phantom.repos
```
Then go back to the root of your ROS 2 workspace and build using:
```sh
colcon build
```

## Usage

### Individual nodes

Once the code is compiled, you can start the ROS 2 Sensable Phantom using:
```sh
ros2 run sensable_phantom sensable_phantom
```

To visualize the Omni in RViz, you will first need to start the robot state publisher:
```sh
ros2 launch sensable_omni_model omni.launch.py
```

And then RViz with the provided configuration file:
```sh
rviz2 -d ~/ros2_ws/install/sensable_omni_model/share/sensable_omni_model/omni.rviz
```

### Launch file with RViz

You can also use the provided launch file
`sensable_phantom_rviz.launch.py` to start all the required nodes for
RViz. `sensable_config` is optional on USB and Ethernet based Omnis,
it's mostly for older, FireWire-based, devices.  See
https://github.com/jhu-saw/sawSensablePhantom.

```sh
cd ~/ros2_ws/install/sawSensablePhantomAll/share/sawSensablePhantom/share
ros2 launch sensable_phantom sensable_phantom_rviz.launch.py sensable_config:=sawSensablePhantomDefault.json
```
