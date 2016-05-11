#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>

#include <graspit_source/include/graspitGUI.h>
#include <graspit_source/include/ivmgr.h>
#include <graspit_source/include/matvec3D.h>

#include "graspit_scene_publisher_plugin.h"
#include "depth_renderer.h"
#include "rgb_renderer.h"
#include "camera_info_builder.h"


namespace graspit_scene_publisher
{

GraspitScenePublisherPlugin::GraspitScenePublisherPlugin() :
    inited(false)
{
}

GraspitScenePublisherPlugin::~GraspitScenePublisherPlugin()
{
    ROS_INFO("ROS GraspIt node stopping");
    ros::shutdown();
}



void GraspitScenePublisherPlugin::publishCameraTF()
{
    transf camera_tf = graspItGUI->getIVmgr()->getCameraTransf();

    static tf::TransformBroadcaster br;

    tf::Transform optical_to_camera;
    optical_to_camera.setIdentity();

    tf::Transform camera_to_world;
    camera_to_world.setIdentity();


    tf::Quaternion optical_to_camera_q;
    optical_to_camera_q.setRPY( M_PI, 0 , 0);
    optical_to_camera.setRotation(optical_to_camera_q);


    tf::Quaternion camera_to_world_q(camera_tf.rotation().x,
                                     camera_tf.rotation().y,
                                     camera_tf.rotation().z,
                                     camera_tf.rotation().w);

    camera_to_world.setRotation(camera_to_world_q);
    camera_to_world.setOrigin( tf::Vector3(camera_tf.translation().x()/1000.0, camera_tf.translation().y()/1000.0,camera_tf.translation().z()/1000.0) );

    tf::Transform t_final = camera_to_world*optical_to_camera.inverse()  ;

    br.sendTransform(tf::StampedTransform(t_final, ros::Time::now(),"/world","/graspit_camera" ));
}

//------------------------- Main class  -------------------------------

int GraspitScenePublisherPlugin::init(int argc, char **argv)
{
    std::string node_name("graspit_scene_publisher");

    ros::init(argc, argv, node_name.c_str());

    root_nh_.reset(new ros::NodeHandle(""));
    priv_nh_.reset(new ros::NodeHandle("~"));

    depthImagePublisher = root_nh_->advertise<sensor_msgs::Image>("/graspit/image_rect",1);
    rgbImagePublisher = root_nh_->advertise<sensor_msgs::Image>("/graspit/image_rect_color",1);
    cameraInfoPublisher = root_nh_->advertise<sensor_msgs::CameraInfo>("/graspit/camera_info",1);

    ROS_INFO("%s Successfully Initialized", node_name.c_str());

    return 0;
}

int GraspitScenePublisherPlugin::mainLoop()
{
    ros::spinOnce();
    if(!inited)
    {
        //this cannot be initialized in init....
        depthRenderer = new DepthRenderer();
        rgbRenderer = new RGBRenderer();
        cameraInfoBuilder = new CameraInfoBuilder();

        inited=true;
    }

    sensor_msgs::CameraInfo *camera_info_msg = new sensor_msgs::CameraInfo;
    cameraInfoBuilder->buildMsg(camera_info_msg);
    cameraInfoPublisher.publish(*camera_info_msg);

    sensor_msgs::Image *rgb_msg = new sensor_msgs::Image;
    rgbRenderer->renderImage(rgb_msg);
    rgbImagePublisher.publish(*rgb_msg);

    sensor_msgs::Image *depth_msg = new sensor_msgs::Image;
    depthRenderer->renderDepthImage(depth_msg);
    depthImagePublisher.publish(*depth_msg);

    publishCameraTF();

    delete depth_msg;
    delete rgb_msg;
    delete camera_info_msg;

    return 0;
}


}
