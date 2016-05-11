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

    double cx =  graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[0] / 2.0;
    double cy =  graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[1] / 2.0;

    double height = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[1];
    double width = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[0];

//    double fx = camera->focalDistance.getValue()/2.0;
//    double fy = camera->focalDistance.getValue()/2.0;

    //fx,fy change with location of camera for some weird reason.
     double fx = 834.451;
     double fy = 834.451;


    std::cout << "fx: " << fx << " fy: " << fy << " cx: " << cx << " cy: " << cy << " height: " << height << " width: " << width << std::endl;

//works mostly
//    double cx =  graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[0] / 2.0;
//    double cy =  graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[1] / 2.0;

//    double height = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[1];
//    double width = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[0];
//    double fx = camera->focalDistance.getValue()/2.0;
//    double fy = camera->focalDistance.getValue()/2.0;

    info->header.stamp = ros::Time::now();
    info->header.frame_id = "/graspit_camera";

//# The image dimensions with which the camera was calibrated. Normally
//# this will be the full camera resolution in pixels.
    info->height = height;
    info->width = width;

//# The distortion model used. Supported models are listed in
//# sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
//# simple model of radial and tangential distortion - is sufficent.
    info->distortion_model = "plub_bob";

//# The distortion parameters, size depending on the distortion model.
//# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    info->D.clear();
    info->D.push_back(0.0);
    info->D.push_back(0.0);
    info->D.push_back(0.0);
    info->D.push_back(0.0);
    info->D.push_back(0.0);

//# Intrinsic camera matrix for the raw (distorted) images.
//#     [fx  0 cx]
//# K = [ 0 fy cy]
//#     [ 0  0  1]
//# Projects 3D points in the camera coordinate frame to 2D pixel
//# coordinates using the focal lengths (fx, fy) and principal point
//# (cx, cy).
    info->K[0] = fx;
    info->K[1] = 0;
    info->K[2] = cx;
    info->K[3] = 0;
    info->K[4] = fy;
    info->K[5] = cy;
    info->K[6] = 0;
    info->K[7] = 0;
    info->K[8] = 1;


//# Rectification matrix (stereo cameras only)
//# A rotation matrix aligning the camera coordinate system to the ideal
//# stereo image plane so that epipolar lines in both stereo images are
//# parallel.
    info->R[0] = 1.0;
    info->R[1] = 0.0;
    info->R[2] = 0.0;
    info->R[3] = 0.0;
    info->R[4] = 1.0;
    info->R[5] = 0.0;
    info->R[6] = 0.0;
    info->R[7] = 0.0;
    info->R[8] = 1.0;

//# Projection/camera matrix
//#     [fx'  0  cx' Tx]
//# P = [ 0  fy' cy' Ty]
//#     [ 0   0   1   0]
//# By convention, this matrix specifies the intrinsic (camera) matrix
//#  of the processed (rectified) image. That is, the left 3x3 portion
//#  is the normal camera intrinsic matrix for the rectified image.
//# It projects 3D points in the camera coordinate frame to 2D pixel
//#  coordinates using the focal lengths (fx', fy') and principal point
//#  (cx', cy') - these may differ from the values in K.
//# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
//#  also have R = the identity and P[1:3,1:3] = K.
//# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
//#  position of the optical center of the second camera in the first
//#  camera's frame. We assume Tz = 0 so both cameras are in the same
//#  stereo image plane. The first camera always has Tx = Ty = 0. For
//#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
//#  Tx = -fx' * B, where B is the baseline between the cameras.
//# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
//#  the rectified image is given by:
//#  [u v w]' = P * [X Y Z 1]'
//#         x = u / w
//#         y = v / w
//#  This holds for both images of a stereo pair.
    info->P[0] = fx;
    info->P[1] = 0.0;
    info->P[2] = cx;
    info->P[3] = 0.0;
    info->P[4] = 0.0;
    info->P[5] = fy;
    info->P[6] = cy;
    info->P[7] = 0.0;
    info->P[8] = 0.0;
    info->P[9] = 0.0;
    info->P[10] = 1.0;
    info->P[11] = 0.0;


//# Binning refers here to any camera setting which combines rectangular
//#  neighborhoods of pixels into larger "super-pixels." It reduces the
//#  resolution of the output image to
//#  (width / binning_x) x (height / binning_y).
//# The default values binning_x = binning_y = 0 is considered the same
//#  as binning_x = binning_y = 1 (no subsampling).
    info->binning_x = 0.0;
    info->binning_y = 0.0;


//# Region of interest (subwindow of full camera resolution), given in
//#  full resolution (unbinned) image coordinates. A particular ROI
//#  always denotes the same window of pixels on the camera sensor,
//#  regardless of binning settings.
//# The default setting of roi (all values 0) is considered the same as
//#  full resolution (roi.width = width, roi.height = height).
    info->roi.x_offset = 0.0;
    info->roi.y_offset = 0.0;
    info->roi.height = 0.0;
    info->roi.width = 0.0;
    info->roi.do_rectify=false;

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

//    delete depthRenderer;
//    delete rgbRenderer;

    return 0;
}


}
