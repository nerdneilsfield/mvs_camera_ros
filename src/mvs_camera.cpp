#include <mvs_camera/mvs_camera.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include "MvCameraControl.h"

namespace mvs_camera {

MvsCamera::MvsCamera(const ros::NodeHandle &nh,
                     const std::string &publish_topic)
    : nh_(nh), publish_topic_(publish_topic) {
  pub_ = nh_.advertise<sensor_msgs::Image>(publish_topic_, 10);
}

MvsCamera::~MvsCamera() { CloseDevice(); }

void MvsCamera::Init() { OpenDevice(); }

void MvsCamera::Start() { StartGrabbing(); }

void MvsCamera::OpenDevice() {
  ROS_INFO("Opening device.....");
  int nRet = MV_OK;
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  // nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    ROS_ERROR("Enum Devices fail! nRet [0x%x]\n", nRet);
    std::exit(1);
  }
  if (stDeviceList.nDeviceNum > 0) {
    ROS_INFO("Found %d devices, use first one", stDeviceList.nDeviceNum);
  } else {
    ROS_ERROR("Find No Devices!\n");
    std::exit(2);
  }

  nRet = MV_CC_CreateHandle(&camera_handle_, stDeviceList.pDeviceInfo[0]);
  if (MV_OK != nRet) {
    ROS_ERROR("Create Handle fail! nRet is [0x%x]\n", nRet);
    std::exit(3);
  }

  nRet = MV_CC_OpenDevice(camera_handle_);
  if (MV_OK != nRet) {
    ROS_ERROR("Open Device fail! nRet [0x%x]", nRet);
    std::exit(4);
  }

  // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal
  // package size(It only works for the GigE camera)
  if (stDeviceList.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE) {
    int nPacketSize = MV_CC_GetOptimalPacketSize(camera_handle_);
    if (nPacketSize > 0) {
      nRet =
          MV_CC_SetIntValue(camera_handle_, "GevSCPSPacketSize", nPacketSize);
      if (nRet != MV_OK) {
        ROS_WARN("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
      }
    } else {
      ROS_WARN("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
    }

    optimal_packet_size_ = nPacketSize;
  }

  // ch:设置触发模式为off | en:Set trigger mode as off
  nRet = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
  if (MV_OK != nRet) {
    ROS_ERROR("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
    std::exit(5);
  }

  // ch:获取数据包大小 | en:Get payload size
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(camera_handle_, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    ROS_ERROR("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    std::exit(6);
  }
  payload_size_ = stParam.nCurValue;

  nRet = MV_CC_StartGrabbing(camera_handle_);
  if (MV_OK != nRet) {
    ROS_ERROR("Start Grabbing fail! nRet [0x%x]\n", nRet);
    std::exit(7);
  }
  ROS_INFO("Camera started!");
}

void MvsCamera::CloseDevice() {
  ROS_INFO("Closing device.....");
  int nRet = 0;
  // ch:停止取流 | en:Stop grab image
  nRet = MV_CC_StopGrabbing(camera_handle_);
  if (MV_OK != nRet) {
    ROS_ERROR("Stop Grabbing fail! nRet [0x%x]\n", nRet);
    std::exit(11);
  }

  // ch:关闭设备 | Close device
  nRet = MV_CC_CloseDevice(camera_handle_);
  if (MV_OK != nRet) {
    ROS_ERROR("ClosDevice fail! nRet [0x%x]\n", nRet);
    std::exit(12);
  }

  // ch:销毁句柄 | Destroy camera_handle_
  nRet = MV_CC_DestroyHandle(camera_handle_);
  if (MV_OK != nRet) {
    ROS_ERROR("Destroy camera_handle_ fail! nRet [0x%x]\n", nRet);
    std::exit(13);
  }

  if (nRet != MV_OK) {
    if (camera_handle_ != NULL) {
      MV_CC_DestroyHandle(camera_handle_);
      camera_handle_ = NULL;
    }
  }
}

void MvsCamera::StartGrabbing() {
  ROS_INFO("Start grabbing image.....");
  MV_FRAME_OUT stOutFrame = {0};
  memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));

  int64_t frame_count = 0;

  while (ros::ok()) {
    int nRet = MV_CC_GetImageBuffer(camera_handle_, &stOutFrame, 1000);
    if (nRet == MV_OK) {
      ROS_DEBUG("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
                stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight,
                stOutFrame.stFrameInfo.nFrameNum);
    } else {
      ROS_WARN("No data[0x%x]\n", nRet);
    }

    sensor_msgs::Image msg;

    msg.header.stamp = ros::Time::now();
    msg.header.seq = frame_count;
    msg.header.frame_id = "camera";

    sensor_msgs::fillImage(msg, "rgb8", stOutFrame.stFrameInfo.nHeight,
                           stOutFrame.stFrameInfo.nWidth,
                           3 * stOutFrame.stFrameInfo.nWidth,
                           stOutFrame.pBufAddr);
    pub_.publish(msg);
    ros::spinOnce();
    frame_count += 1;

    if (NULL != stOutFrame.pBufAddr) {
      nRet = MV_CC_FreeImageBuffer(camera_handle_, &stOutFrame);
      if (nRet != MV_OK) {
        ROS_ERROR("Free Image Buffer fail! nRet [0x%x]\n", nRet);
      }
    }
  }

  CloseDevice();
}

} // namespace mvs_camera
