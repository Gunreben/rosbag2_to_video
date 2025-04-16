# rosbag2_to_video Tool (Fork with GUI)

**Note:** This is a fork of the original [fictionlab/rosbag2_to_video](https://github.com/fictionlab/rosbag2_to_video) repository, adding a graphical user interface (GUI) for easier topic selection and export.

## Installation

### Binaries (Original Tool)

```bash
sudo apt install ros-${ROS_DISTRO}-rosbag2-to-video
```

### From source (This Fork)

To use the version with the GUI, install from source:

```bash
# create workspace
mkdir -p your_ros2_ws/src
cd your_ros2_ws/src
# clone this repo
git clone https://github.com/Gunreben/rosbag2_to_video.git
cd ..
# install dependencies (includes PySide6 and qt-material for GUI)
rosdep update
rosdep install --from-paths src --ignore-src -y
# If rosdep fails for qt-material, install it via pip:
# python3 -m pip install qt-material

# build workspace
colcon build
# source the workspace
source install/setup.bash
```

## Usage

### Command Line Interface (Original)

The original command-line tool is still available:

```bash
ros2 run rosbag2_to_video rosbag2_to_video --help
```

It also integrates with `ros2 bag`:

```bash
ros2 bag to_video --help
```

### Graphical User Interface (GUI - Added in this Fork)

After building and sourcing the workspace (from the "From source" instructions), run the GUI using:

```bash
ros2 run rosbag2_to_video rosbag2_to_video_gui
```

This will open a window allowing you to browse for a bag file, select image topics, choose an output directory/codec/FPS, and export videos.
