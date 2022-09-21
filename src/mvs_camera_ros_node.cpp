#include <ros/ros.h>

#include "mvs_camera/mvs_camera.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mvs_camera_ros_node");

  ros::NodeHandle nh("~");

  mvs_camera::MvsCamera camera(nh, "camera");

  camera.Init();

  camera.Start();
}
