#ifndef INCLUDE_MVS_CAMREA_H_
#define INCLUDE_MVS_CAMREA_H_

#include "MvCameraControl.h"
#include <ros/ros.h>

#include <cstdint>
#include <memory>
#include <string>

namespace mvs_camera {

class MvsCamera {
public:
  MvsCamera(const ros::NodeHandle &nh, const std::string &publish_topic);
  ~MvsCamera();
  void Init();
  void Start();

private:
  void OpenDevice();
  void CloseDevice();
  void StartGrabbing();
  // void GrabOneFrame();
  // void Publish();

private:
  std::string publish_topic_;

  uint8_t *image_data_;
  unsigned int payload_size_ = 0;
  int optimal_packet_size_ = 0;

  void *camera_handle_ = nullptr;

  ros::Publisher pub_;
  ros::NodeHandle nh_;
};

} // namespace mvs_camera

#endif // INCLUDE_MVS_CAMREA_H_
