#include <depth_renderer.h>
#include <QImage>

#include <rospack/rospack.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <graspit_source/include/graspitGUI.h>
#include <graspit_source/include/ivmgr.h>

#include <math.h>


DepthRenderer::DepthRenderer()
{

    renderArea = graspItGUI->getIVmgr()->getViewer();

    depthBuffer = new SoDepthBuffer;
    depthBuffer->function = SoDepthBufferElement::LESS;
    depthBuffer->test = true;
    depthBuffer->write = true;

    ROS_INFO_STREAM("About to get Render Scripts Path");
    std::string s2;
    static rospack::Rospack rp;
    std::vector<std::string> sp;
    rp.getSearchPathFromEnv(sp);
    rp.crawl(sp, true);
    rp.find("graspit_scene_publisher", s2);
    std::string script_path = s2 + std::string("/render_scripts");
    ROS_INFO_STREAM("Loading Render Scripts from " << script_path << std::endl);

    vertexShader = new SoVertexShader;
    vertexShader->sourceProgram.setValue((script_path + "/depth_vert.glsl").c_str());

    fragmentShader = new SoFragmentShader;
    fragmentShader->sourceProgram.setValue((script_path + "/depth_frag.glsl").c_str());

    shaderProgram = new SoShaderProgram;
    shaderProgram->shaderObject.set1Value(0, vertexShader);
    shaderProgram->shaderObject.set1Value(1, fragmentShader);

    shaderSep = new SoSeparator;
    shaderSep->ref();
    shaderSep->addChild(depthBuffer);
    shaderSep->addChild(shaderProgram);
    shaderSep->addChild(graspItGUI->getIVmgr()->getViewer()->getCamera());
    shaderSep->addChild(renderArea->getSceneGraph());

    glRend = new SoGLRenderAction(renderArea->getViewportRegion());
    glRend->setSmoothing(false);
    glRend->setNumPasses(5);
    glRend->setTransparencyType(SoGLRenderAction::NONE);

    const SbColor white(0, 0, 0);
    myRenderer = new SoOffscreenRenderer(glRend);
    myRenderer->setBackgroundColor(white);
    myRenderer->setComponents(SoOffscreenRenderer::RGB_TRANSPARENCY);

    verbose = false;
}

DepthRenderer::~DepthRenderer()
{
    shaderSep->unref();
    delete renderArea;
    delete glRend;
    delete myRenderer;
    delete vertexShader;
    delete fragmentShader;
}



void DepthRenderer::renderDepthImage(sensor_msgs::Image * img_ptr)
{
    myRenderer->render(shaderSep);

    SbBool result;
    result = myRenderer->writeToFile("test","png");


    unsigned char * data = myRenderer->getBuffer();
    if (verbose)
    {
      ROS_ERROR_STREAM("Test char 0:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[0]);
      ROS_ERROR_STREAM("Test char 1:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[1]);
      ROS_ERROR_STREAM("Test char 2:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[2]);
      ROS_ERROR_STREAM("Test char 3:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[3]);
    }

    float scale_factor = 10000.0; // output units to meters conversion factor

    if(img_ptr != NULL)
    {
      img_ptr->height = myRenderer->getViewportRegion().getWindowSize()[1];
      img_ptr->width = myRenderer->getViewportRegion().getWindowSize()[0];
      img_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      img_ptr->data.resize(img_ptr->height*img_ptr->width*sizeof(float));
      img_ptr->step = img_ptr->width*sizeof(float);
      img_ptr->is_bigendian = false;
      img_ptr->header.frame_id = "/graspit_camera";
      img_ptr->header.stamp = ros::Time::now();

       //std::cout << "depth_image height: " << img_ptr->height << " width: " << img_ptr->width << std::endl;

      float * float_data = (float*)img_ptr->data.data();

       int total_size =  img_ptr->data.size();
       int width = img_ptr->width;
       int height = img_ptr->height;

       size_t i = 0;
       int index = 0;
       for(int h = 0; h < height; ++ h)
       {
           //250
           for(int w = 0; w < width; ++ w)
           {
               //working
               //index = i/4;
               //index = h*width + w;
               index = (height-h-1)*width + w;

               int val = 0;
               for (int c =0; c < 3; c++)
               {
                   val += data[i]*pow(255,(3-c-1));
                   ++i;
               }
               ++i; // extra one because 4 channels...
               //std::cout << "w " << w << " width " << width << "index" << index <<std::endl;
               float_data[index] =  val/scale_factor;
           }
       }

       //works but flipped
//      for(size_t i = 0; i < img_ptr->data.size()/sizeof(float); ++ i)
//      {
//          float_data[i] = (data[i*4]*255*255 + data[i*4 + 1]*255 + data[i*4 + 2])/scale_factor;
//     }

    }
}
