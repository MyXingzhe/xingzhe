#include <ros/ros.h>
#include <nodelet/loader.h>

#include "camera.h"

int main (int argc, char **argv) {
  ros::init(argc, argv, "uvc_camera");

  uvc_camera::Camera camera(ros::NodeHandle(), ros::NodeHandle("~"));

  ros::spin();
  return 0;
}

