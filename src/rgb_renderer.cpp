#include <rgb_renderer.h>
#include <QImage>
#include <QGLWidget>
#include <graspit_source/include/graspitGUI.h>
#include <graspit_source/include/ivmgr.h>


RGBRenderer::RGBRenderer()
{
    renderArea = graspItGUI->getIVmgr()->getViewer();
}

RGBRenderer::~RGBRenderer()
{
    delete renderArea;
}


void RGBRenderer::renderImage(sensor_msgs::Image * img_ptr)
{
    QGLWidget * glWidget = dynamic_cast<QGLWidget *>(renderArea->getGLWidget());
    QImage image = glWidget->grabFrameBuffer();
    QImage::Format f = image.format();

    image = image.convertToFormat(QImage::Format_RGB32);

        int size_of_elem = 3;

        if(img_ptr != NULL)
        {
            img_ptr->height = renderArea->getViewportRegion().getWindowSize()[1];
            img_ptr->width = renderArea->getViewportRegion().getWindowSize()[0];
            img_ptr->encoding = "rgb8" ;//sensor_msgs::image_encodings::RGB8;
            img_ptr->data.resize(img_ptr->height*img_ptr->width*size_of_elem);
            img_ptr->step = img_ptr->width*size_of_elem;
            img_ptr->is_bigendian = false;
            img_ptr->header.frame_id = "/camera_rgb_optical_frame";
            img_ptr->header.stamp = ros::Time::now();

              int channels = 3;
              unsigned char * sensor_data = (unsigned char *)img_ptr->data.data();

              for(int y = 0; y < img_ptr->height; ++y, sensor_data+=img_ptr->width*size_of_elem)
              {
                  for(int x = 0; x < img_ptr->width; ++x)
                  {
                      QRgb rgb = image.pixel(x, y);
                      sensor_data[x * channels] = qRed(rgb); //r
                      sensor_data[x * channels+1] = qGreen(rgb); //g
                      sensor_data[x * channels+2] = qBlue(rgb); //b
                  }
              }

        }
}
