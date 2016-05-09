#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>

#include <graspit_source/include/graspitGUI.h>
#include <graspit_source/include/ivmgr.h>
#include <graspit_source/include/matvec3D.h>

#include "graspit_pointcloud_pub.h"
#include "depth_renderer.h"
#include "rgb_renderer.h"


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

void GraspitPointCloudPub::getCameraInfoFromCamera(sensor_msgs::CameraInfo * info)
{
    SoPerspectiveCamera * camera = static_cast<SoPerspectiveCamera* >(graspItGUI->getIVmgr()->getViewer()->getCamera());

    info->header.stamp = ros::Time::now();
    info->header.frame_id = "/graspit_camera";

    info->height = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[1];
    info->width = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[0];
    info->distortion_model = "plub_bob";

    //enabling D, segfaults for some reason.
//    info->D[0] = 0.0;
//    info->D[1] = 0.0;
//    info->D[2] = 0.0;
//    info->D[3] = 0.0;
//    info->D[4] = 0.0;

    info->K[0] = camera->focalDistance.getValue(); //fx
    info->K[1] = 0;
    info->K[2] = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[0]/2; //cx
    info->K[3] = 0;
    info->K[4] = camera->focalDistance.getValue(); //fy
    info->K[5] = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[1]/2; //cy
    info->K[6] = 0;
    info->K[7] = 0;
    info->K[8] = 1;

    info->R[0] = 1.0;
    info->R[1] = 0.0;
    info->R[2] = 0.0;
    info->R[3] = 0.0;
    info->R[4] = 1.0;
    info->R[5] = 0.0;
    info->R[6] = 0.0;
    info->R[7] = 0.0;
    info->R[8] = 1.0;

    info->P[0] = camera->focalDistance.getValue(); //fx
    info->P[1] = 0.0;
    info->P[2] = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[0]/2; //cx
    info->P[3] = 0.0;
    info->P[4] = 0.0;
    info->P[5] = camera->focalDistance.getValue(); //fy
    info->P[6] =  graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[1]/2; //cy
    info->P[7] = 0.0;
    info->P[8] = 0.0;
    info->P[9] = 0.0;
    info->P[10] = 1.0;
    info->P[11] = 0.0;

    info->binning_x = 0.0;
    info->binning_y = 0.0;

    info->roi.x_offset = 0.0;
    info->roi.y_offset = 0.0;
    info->roi.height = 0.0;
    info->roi.width = 0;
    info->roi.do_rectify=false;

}

void GraspitPointCloudPub::setCameraFromInfo(sensor_msgs::CameraInfo & info, SoPerspectiveCamera * cam)
{
      double focal_length = info.K[0];
      double fovy =  2 * atan(info.height / (2 * focal_length)) * 180.0 / M_PI;
      cam->heightAngle = fovy;
      cam->focalDistance = focal_length;
      // These numbers are arbitrary
      cam->nearDistance = 3;
      cam->farDistance = 2000;
}


void GraspitPointCloudPub::publishCameraTF()
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

int GraspitPointCloudPub::init(int argc, char **argv)
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

int GraspitPointCloudPub::mainLoop()
{
    ros::spinOnce();
    if(!inited)
    {
        //this cannot be initialized in init....
        depthRenderer = new DepthRenderer();
        rgbRenderer = new RGBRenderer();
        inited=true;
    }

    sensor_msgs::CameraInfo *camera_info_msg = new sensor_msgs::CameraInfo;
    getCameraInfoFromCamera(camera_info_msg);
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
