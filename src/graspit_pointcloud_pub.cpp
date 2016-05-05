#include "graspit_pointcloud_pub.h"
#include "depth_renderer.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <rospack/rospack.h>
#include <ros/package.h>

namespace graspit_pointcloud_pub
{

GraspitPointCloudPub::GraspitPointCloudPub() :
    inited(false)
{
}

GraspitPointCloudPub::~GraspitPointCloudPub()
{
    ROS_INFO("ROS GraspIt node stopping");
    ros::shutdown();
}

//------------------------- Main class  -------------------------------

int GraspitPointCloudPub::init(int argc, char **argv)
{
    std::string node_name("graspit_scene_publisher");

    ros::init(argc, argv, node_name.c_str());

    //    //init node handles
    root_nh_.reset(new ros::NodeHandle(""));
    priv_nh_.reset(new ros::NodeHandle("~"));

    ROS_INFO("%s Successfully Initialized", node_name.c_str());

    return 0;
}

int GraspitPointCloudPub::mainLoop()
{

    ros::spinOnce();
    if(!inited)
    {
        depthImagePublisher = root_nh_->advertise<sensor_msgs::Image>("/graspit/image_rect",1);
        rgbImagePublisher = root_nh_->advertise<sensor_msgs::Image>("/graspit/image_rect_color",1);
        cameraInfoPublisher = root_nh_->advertise<sensor_msgs::CameraInfo>("/graspit/camera_info",1);

        depthRenderer = new DepthRenderer();
        inited=true;
    }



   sensor_msgs::CameraInfo *camera_info_msg = new sensor_msgs::CameraInfo;
   depthRenderer->getCameraInfoFromCamera(camera_info_msg);
   cameraInfoPublisher.publish(*camera_info_msg);

   sensor_msgs::Image *rgb_msg = new sensor_msgs::Image;
   depthRenderer->renderImage(rgb_msg);
   rgbImagePublisher.publish(*rgb_msg);

//    sensor_msgs::Image *depth_msg = new sensor_msgs::Image;
//    depthRenderer->renderDepthImage(depth_msg);
//    depthImagePublisher.publish(*depth_msg);


//    //delete depth_msg;
//    delete rgb_msg;
//    delete camera_info_msg;

    return 0;
}


}
