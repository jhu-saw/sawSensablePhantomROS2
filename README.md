# sawSensablePhantomROS2

ROS 2 node for sawSensablePhantom (https://github.com/jhu-saw/sawSensablePhantom).

First install dependencies for cisst/SAW:
```sh
sudo apt install python3-vcstool python3-colcon-common-extensions libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev
```

You can use the ROS 2 VCS python-based tool (`sudo apt install python3-vcstool`) to download all the cisst, sawSensablePhantom and ROS 2 specific packages needed for the sawSensablePhantom ROS 2 node.  Look for the `.repos` file in this repository and find the link to the raw content.  Then in your ROS 2 workspace, under the `src` directory, use:
```sh
vcs import --input https://raw.githubusercontent.com/jhu-saw/sawSensablePhantomROS2/main/sensable_phantom.repos
```
Then go back to the root of your ROS 2 workspace and build using:
```sh
colcon build
```

Once the code is compiled, you can start the ROS 2 NDI tracker using:
```
ros2 run sensable_phantom sensable_phantom
```
