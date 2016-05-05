#include "graspit_pointcloud_pub.h"
#include "depth_renderer.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace graspit_pointcloud_pub
{

GraspitPointCloudPub::GraspitPointCloudPub() :
    root_nh_(NULL),
    priv_nh_(NULL),
    inited(false)
{
}

GraspitPointCloudPub::~GraspitPointCloudPub()
{
    ROS_INFO("ROS GraspIt node stopping");
    ros::shutdown();
    delete root_nh_;
    delete priv_nh_;
}

//------------------------- Main class  -------------------------------

int GraspitPointCloudPub::init(int argc, char **argv)
{
    //copy the arguments somewhere else so we can pass them to ROS
    int ros_argc = argc;
    char** ros_argv = new char*[argc];
    for (int i = 0; i < argc; i++)
    {
        ros_argv[i] = new char[strlen(argv[i])];
        strcpy(ros_argv[i], argv[i]);
    }
    //see if a node name was requested
    std::string node_name("graspit_pointcloud_publisher");
    for (int i = 0; i < argc - 1; i++)
    {
        //std::cerr << argv[i] << "\n";
        if (!strcmp(argv[i], "_name"))
        {
            node_name = argv[i + 1];
        }
    }
    //init ros
    ros::init(ros_argc, ros_argv, node_name.c_str());

//    //init node handles
    root_nh_ = new ros::NodeHandle("");
    priv_nh_ = new ros::NodeHandle("~");


    ROS_INFO("Using node name %s", node_name.c_str());
    for (int i = 0; i < argc; i++)
    {
        delete ros_argv[i];
    }
    delete ros_argv;

    ROS_INFO("Graspit Point Cloud publisher Successfully Initialized");
    return 0;
}

int GraspitPointCloudPub::mainLoop()
{
    if(!inited)
    {
        depthImagePublisher = root_nh_->advertise<sensor_msgs::Image>("/graspit/image_rect",1);
        rgbImagePublisher = root_nh_->advertise<sensor_msgs::Image>("/graspit/image_rect_color",1);
        cameraInfoPublisher = root_nh_->advertise<sensor_msgs::CameraInfo>("/graspit/camera_info",1);

        depthRenderer = new DepthRenderer();
        inited=true;
    }

    ros::spinOnce();

   sensor_msgs::CameraInfo *camera_info_msg = new sensor_msgs::CameraInfo;
   depthRenderer->getCameraInfoFromCamera(camera_info_msg);
   cameraInfoPublisher.publish(*camera_info_msg);

   sensor_msgs::Image *rgb_msg = new sensor_msgs::Image;
   depthRenderer->renderImage(rgb_msg);
   rgbImagePublisher.publish(*rgb_msg);

//    sensor_msgs::Image *depth_msg = new sensor_msgs::Image;
//    depthRenderer->renderDepthImage(depth_msg);
//    depthImagePublisher.publish(*depth_msg);


    //delete depth_msg;
    delete rgb_msg;
    delete camera_info_msg;

    return 0;
}


}
