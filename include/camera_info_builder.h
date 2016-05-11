#ifndef _CAMERA_INFO_BUILDER_H_
#define _CAMERA_INFO_BUILDER_H_

#include <sensor_msgs/CameraInfo.h>

class CameraInfoBuilder
{

public:
    void buildMsg(sensor_msgs::CameraInfo * info);
};

#endif
