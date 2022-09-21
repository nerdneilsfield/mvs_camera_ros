# mvs_camera_ros
A very simple ros driver for mvs_camera_ros

> Noted: Must `source /opt/MVS/bin/set_env_path.sh` before run the node


> Currently, the driver use the embedded configuration in Camera, you can use MVS to configurate it.

## Usage

0. Install MVS tool
1. Clone the repo to your workspace
2. Build the package
3. `source /opt/MVS/bin/set_env_path.sh`
4. `rosrun mvs_camera_ros mvs_camera_ros_node`
5. Open a `rqt_image_view` to see the result


## Planning

- [ ] change the `node` to `nodelet`
- [ ] read the camera configurations from file

