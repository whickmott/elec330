# Jetbot — ROS 2 Jazzy

Assuming students have installed ros-dev-tools, ros desktop and gazebo

## Dependencies

```bash
sudo apt install \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-gscam \
    ros-jazzy-rqt-image-view \
    python3-smbus \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good
```

## Jetson camera setup (once)

For Raspberry Pi Cam v2.1 (IMX219) on the Orin

```bash
sudo /opt/nvidia/jetson-io/config-by-hardware.py -n 2="Camera IMX219 Dual" &&
sudo reboot
```

After reconnecting, check the camera is detected:

```bash
ls -l /dev/video*
sudo dmesg | grep -i imx219
```

Expect `/dev/video0` and a bound IMX219 sensor. With one camera connected, the unused port may report a failed probe but means theyre ready to go if we choose to use 2!

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --base-paths src
source install/setup.bash
```

## Launch

Physical motors with encoder feedback in RViz and Gazebo:

```bash
ros2 launch my_robot_bringup encoder_demo.launch.xml motors:=true
```

Manual encoder turns controlling simulated wheels:

```bash
ros2 launch my_robot_bringup encoder_demo.launch.xml
```

Keyboard teleop, in another terminal while the motor launch runs:

```bash
ros2 run my_robot_bringup teleop.py
```

| Key | Action |
|---|---|
| W / S | Forwards / backwards |
| A / D | Turn left / right |
| Space | Stop |
| Q | Exit teleop |

Camera with viewer:

```bash
ros2 launch pi_camera camera.launch.xml
```

Camera without viewer:

```bash
ros2 launch pi_camera camera.launch.xml viewer:=false
```

Images are published on `/camera/image_raw`
