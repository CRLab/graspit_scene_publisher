#ifndef _GRASPIT_POINTCLOUD_PUB_H_
#define _GRASPIT_POINTCLOUD_PUB_H_

#include <include/plugin.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class DepthRenderer;

namespace graspit_pointcloud_pub
{

class GraspitPointCloudPub :  public Plugin
{

private:

  //! Node handle in the root namespace
  ros::NodeHandlePtr root_nh_;
  //! Node handle in the private namespace
  ros::NodeHandlePtr priv_nh_;

  ros::Publisher depthImagePublisher;
  ros::Publisher rgbImagePublisher;
  ros::Publisher cameraInfoPublisher;

   DepthRenderer *depthRenderer;

   bool inited;


public:
 
  GraspitPointCloudPub();
  ~GraspitPointCloudPub();

  virtual int init(int argc, char **argv);
  virtual int mainLoop();
};


}

#endif //_GRASPIT_POINTCLOUD_PUB_H_
