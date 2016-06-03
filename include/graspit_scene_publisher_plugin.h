#ifndef _GRASPIT_POINTCLOUD_PUB_H_
#define _GRASPIT_POINTCLOUD_PUB_H_

#include <include/plugin.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>

class DepthRenderer;
class RGBRenderer;
class CameraInfoBuilder;
class SoPerspectiveCamera;

namespace graspit_scene_publisher
{

class GraspitScenePublisherPlugin :  public Plugin
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
   RGBRenderer *rgbRenderer;
   CameraInfoBuilder *cameraInfoBuilder;

   sensor_msgs::CameraInfo *camera_info_msg;
   sensor_msgs::Image *rgb_msg;
   sensor_msgs::Image *depth_msg;

   ros::Time time_of_last_publish;


   bool inited;

   void publishCameraTF();


public:
 
  GraspitScenePublisherPlugin();
  ~GraspitScenePublisherPlugin();

  virtual int init(int argc, char **argv);
  virtual int mainLoop();
};


}

#endif //_GRASPIT_POINTCLOUD_PUB_H_
