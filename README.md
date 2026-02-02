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

### Sonia packages

* `sonia_common_ros2`

### External packages

* `Boost`
* `OpenCV`
* `Spinnaker SDK`

#### Spinnaker SDK Installation
**cam_port_manager** requires Spinnaker SDK to build and run, the project contains all the installation files for spinnaker version `4.2` in `drivers/`.

From within the folder `drivers/`, chose the right subfolder, depending on the archetecture of the system used: `x86` or `arm64`. Follow the instructions in the `README.md` placed in the folder for a installation of spinnaker.

---

## Node

* Name: `cam_provider`
* Port Name: `/dev/CAM0`
---

## Registered Topics / Services / Actions

| Type            | Name                           | Direction       | Message/Service Type                | Description                                               |
| --------------- | ------------------------------ | ----------------| ----------------------------------- | --------------------------------------------------------  |
| Topic           | `/system_monitor/node_status`  | Published       | `sonia_common_ros2/msg/NodeStatus`  | Message contains information of the state of a node       |

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

* [sonia_common_ros2](https://github.com/sonia-auv/sonia_common_ros2)
* [Spinnaker SDK](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/?model=Spinnaker%20SDK&vertical=machine%20vision&segment=iis)
* [Flir Chameleon3](https://www.teledynevisionsolutions.com/en-ca/products/chameleon3-usb3/?model=CM3-U3-31S4C-CS&vertical=machine%20vision&segment=iis)

---