# cam_port_manager

*description*

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
* `Spinnaker SDK`

---

## Node

* Name: ``
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

---