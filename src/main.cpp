
#include "graspit_pointcloud_pub.h"

extern "C" Plugin* createPlugin() {
  return new graspit_pointcloud_pub::GraspitPointCloudPub();
}

extern "C" std::string getType() {
  return "graspit_pointcloud_pub";
}
