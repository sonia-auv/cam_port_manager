# cam_port_manager

The project opens up a connection with 1 or 2 **Flir Chameleon3** cameras. The feed from the camera is transported using **image_transport** on the **ROS2** network. **image_transport** handles the creation of the required topics and publishes, raw feed, compressed feed and information from the camera.

---

## Dependencies

### ROS 2 Distro

* Humble

### ROS 2 Packages

* `ament_cmake`
* `rclcpp`
* `image_transport`
* `cv_bridge`
* `sensor_msgs`

### External packages

* `Boost`
* `OpenCV`
* `Spinnaker SDK`

---

## Node

* Name: `cam_provider`
* Port Name: `/dev/CAM0`

---
## Build Instructions
To build the project, the following commands should be run directly from your ROS2 workspace.

```bash
colcon build --packages-select cam_port_manager --symlink-install
source install/setup.bash
```

---

## Launch Instructions

### Environment variables
Required environment variables to launch the project

```bash
export AUV={prototype_identifier}
```
replace `{prototype_identifier}` with available options: `AUV8` | `AUV7`.

### Default launch

```bash
ros2 launch cam_port_manager launch.py
```

---

## Useful ROS 2 Commands

```bash
ros2 node list
ros2 node info /cam_provider
ros2 param list /cam_provider
```

---

## References

* [Spinnaker SDK](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/?model=Spinnaker%20SDK&vertical=machine%20vision&segment=iis)
* [Flir Chameleon3](https://www.teledynevisionsolutions.com/en-ca/products/chameleon3-usb3/?model=CM3-U3-31S4C-CS&vertical=machine%20vision&segment=iis)

---