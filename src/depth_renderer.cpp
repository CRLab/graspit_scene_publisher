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

    const SbColor black(255, 255, 255);
    myRenderer = new SoOffscreenRenderer(glRend);
    myRenderer->setBackgroundColor(black);
    myRenderer->setComponents(SoOffscreenRenderer::RGB_TRANSPARENCY);
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

    unsigned char * data = myRenderer->getBuffer();

    float scale_factor = 1000.0; // output units to meters conversion factor

    if(img_ptr != NULL)
    {
      img_ptr->height = myRenderer->getViewportRegion().getWindowSize()[1];
      img_ptr->width = myRenderer->getViewportRegion().getWindowSize()[0];
      img_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      img_ptr->data.resize(img_ptr->height*img_ptr->width*sizeof(float));
      img_ptr->step = img_ptr->width*sizeof(float);
      img_ptr->is_bigendian = false;
      img_ptr->header.frame_id = "/camera_rgb_optical_frame";
      img_ptr->header.stamp = ros::Time::now();

      float * float_data = (float*)img_ptr->data.data();

       int width = img_ptr->width;
       int height = img_ptr->height;

       size_t i = 0;
       int index = 0;
       for(int h = 0; h < height; ++ h)
       {
           for(int w = 0; w < width; ++ w)
           {
               index = (height-h-1)*width + w;

               int val = 0;
               for (int c =0; c < 3; c++)
               {
                   val += data[i]*pow(255,(3-c-1)) ;
                   ++i;
               }
               ++i; // extra one because 4 channels...
               float_data[index] =  val/ scale_factor;
           }
       }

    }
}
