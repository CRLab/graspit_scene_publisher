#include <depth_renderer.h>
#include <QImage>
#include <graspit_source/include/graspitGUI.h>
#include <graspit_source/include/ivmgr.h>


void DepthRenderer::getCameraInfoFromCamera(sensor_msgs::CameraInfo * info)
{
    SoPerspectiveCamera * camera = static_cast<SoPerspectiveCamera* >(graspItGUI->getIVmgr()->getViewer()->getCamera());
    info->K[0] = camera->focalDistance.getValue();
    info->K[4] = camera->focalDistance.getValue();
    info->height = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[1];
    info->width = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[0];
    info->K[2] = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[0]/2;
    info->K[5] = graspItGUI->getIVmgr()->getViewer()->getViewportRegion().getViewportSizePixels()[1]/2;
    info->K[8] = 1;

}

void setCameraFromInfo(sensor_msgs::CameraInfo & info, SoPerspectiveCamera * cam)
{
      //SoPerspectiveCamera * cam = static_cast<SoPerspectiveCamera* >(getIVmgr()->getViewer()->getCamera());
      double focal_length = info.K[0];
      double fovy =  2 * atan(info.height / (2 * focal_length)) * 180.0 / M_PI;
      cam->heightAngle = fovy;
      cam->focalDistance = focal_length;
      // These numbers are arbitrary
      cam->nearDistance = 3;
      cam->farDistance = 2000;
}


DepthRenderer::DepthRenderer()
{

    renderArea = graspItGUI->getIVmgr()->getViewer();

    depthBuffer = new SoDepthBuffer;
    depthBuffer->function = SoDepthBufferElement::LESS;
    depthBuffer->test = true;
    depthBuffer->write = true;

    //std::string script_path = ros::package::getPath("graspit_pointcloud_pub") + std::string("/render_scripts");
    std::string script_path = std::string("/home/jalapeno/curg/tactile_simulation_graspit_ws/src/graspit_pointcloud_pub/render_scripts");

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

    //for rgb render
    sceneTex = new SoReadableSceneTexture2;
    sceneTex->type = SoReadableSceneTexture2::RGBA32F;
    sceneTex->size.setValue(512,512);
    sceneTex->type = SoSceneTexture2::RGBA32F;
    sceneTex->ref();
    sceneTex->scene = shaderSep;
    sceneTex->model.setValue(SoSceneTexture2::NONE);
    sceneTex->GLRender(glRend);

     spy = new SoGLFrameBufferSpy;

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


//void renderImage(sensor_msgs::ImagePtr & img_ptr)
//{
//    SoQtRenderArea *renderArea;
//    SoGLRenderAction *glRend;
//    SoOffscreenRenderer *myRenderer;
//    SoVertexShader *vertexShader = new SoVertexShader;
//    SoFragmentShader *fragmentShader = new SoFragmentShader;
//    SoShaderProgram *shaderProgram = new SoShaderProgram;
//    SoSeparator *sceneSep = new SoSeparator;
//    SoDepthBuffer * depthBuffer = new SoDepthBuffer;
//    const SbColor white(0, 0, 0);

//    sceneSep->ref();

//    renderArea = getIVmgr()->getViewer();

//    depthBuffer->function = SoDepthBufferElement::LESS;
//    depthBuffer->test = true;
//    depthBuffer->write = true;


//    glRend = new SoGLRenderAction(renderArea->getViewportRegion());
//    glRend->setSmoothing(false);
//    glRend->setNumPasses(5);
//    glRend->setTransparencyType(SoGLRenderAction::NONE);

//    myRenderer = new SoOffscreenRenderer(glRend);
//    myRenderer->setBackgroundColor(white);

//#ifdef GRASPITDBG
//    if (myRenderer->isWriteSupported("jpg"))
//        std::cout << " supports jpg" << std::endl;
//    else
//        std::cout << "no jpg support" << std::endl;
//#endif

//    myRenderer->setComponents(SoOffscreenRenderer::RGB_TRANSPARENCY);
//    myRenderer->render(sceneSep);;

//    SbBool result;
//    //result = myRenderer->writeToFile("test","png");

//    unsigned char * test = myRenderer->getBuffer();
//    sceneSep->addChild(depthBuffer);
//    sceneSep->addChild(getIVmgr()->getViewer()->getCamera());
//    sceneSep->addChild(renderArea->getSceneGraph());

//    sceneSep->unref();
//}


void DepthRenderer::renderImage(sensor_msgs::Image * img_ptr)
{

    const SbColor white(0, 0, 0);

    myRenderer->render(shaderSep);
    glRend->apply(sceneTex);

    SbBool result;
    result = myRenderer->writeToFile("test","png");

    unsigned char * data = myRenderer->getBuffer();

    float scale_factor = 10000.0;
    if(img_ptr != NULL)
    {
      img_ptr->height = myRenderer->getViewportRegion().getWindowSize()[1];
      img_ptr->width = myRenderer->getViewportRegion().getWindowSize()[0];
      img_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      img_ptr->data.resize(img_ptr->height*img_ptr->width*sizeof(float));
      img_ptr->step = img_ptr->width*sizeof(float);
      img_ptr->is_bigendian = false;
      img_ptr->header.frame_id = "/camera";
      img_ptr->header.stamp = ros::Time::now();

      float * float_data = (float*)img_ptr->data.data();

      //std::cout << " data size: " << img_ptr->data.size() << std::endl;

      for(size_t i = 0; i < img_ptr->data.size()/sizeof(float); ++ i)
      {
          float_data[i] = (data[i*4]*255*255 + data[i*4 + 1]*255 + data[i*4 + 2])/scale_factor;
          if (verbose)
              std::cout << " float_data: " << float_data[i] << std::endl;
      }
    }
//    sceneSep->addChild(depthBuffer);
//    sceneSep->addChild(graspItGUI->getIVmgr()->getViewer()->getCamera());
//    sceneSep->addChild(renderArea->getSceneGraph());

//    sceneSep->unref();
}

//void DepthRenderer::renderDepthImage(sensor_msgs::Image * img_ptr)
//{
//    myRenderer->render(shaderSep);

//    SbBool result;
//    result = myRenderer->writeToFile("test","png");
//    ROS_INFO_STREAM("Saving /home/jalapeno/curg/tactile_simulation_graspit_ws/test.png result:" << result);

//    unsigned char * data = myRenderer->getBuffer();
//    if (verbose)
//    {
//        ROS_ERROR_STREAM("Test char 0:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[0]);
//        ROS_ERROR_STREAM("Test char 1:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[1]);
//        ROS_ERROR_STREAM("Test char 2:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[2]);
//        ROS_ERROR_STREAM("Test char 3:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[3]);
//    }

//    float scale_factor = 10000.0; // output units to meters conversion factor

//    int i = 0;
//    double max=0;
//    double min=0;

//    int rendered_height = myRenderer->getViewportRegion().getWindowSize()[1];
//    int rendered_width = myRenderer->getViewportRegion().getWindowSize()[0];

//    for (int height_idx = 0; height_idx < rendered_height; ++height_idx) {
//        for (int width_idx = 0; width_idx < rendered_width; ++width_idx) {
//            float value = (data[i*4]*255*255 + data[i*4 + 1]*255 + data[i*4 + 2]);
//            if (value > max) max = value;
//            if (value < min) min = value;
//            i++;
//        }
//    }

//    int scaled_height = 480;
//    int scaled_width = 640;

//    if(img_ptr != NULL)
//    {
//        img_ptr->height = scaled_height; // myRenderer->getViewportRegion().getWindowSize()[1];
//        img_ptr->width = scaled_width; //myRenderer->getViewportRegion().getWindowSize()[0];
//        img_ptr->encoding = sensor_msgs::image_encodings::TYPE_8UC3;
//        img_ptr->data.resize(img_ptr->height*img_ptr->width*sizeof(float));
//        img_ptr->step = img_ptr->width*sizeof(float);
//        img_ptr->is_bigendian = false;
//        img_ptr->header.frame_id = "/camera";
//        img_ptr->header.stamp = ros::Time::now();

//        float * float_data = (float*)img_ptr->data.data();

//        QImage *img = new QImage(rendered_width, rendered_height, QImage::Format_RGB32);

//        i = 0;
//        for (int height_idx = 0; height_idx < rendered_height; ++height_idx) {
//            for (int width_idx = 0; width_idx < rendered_width; ++width_idx) {
//                float value = (data[i*4]*255*255 + data[i*4 + 1]*255 + data[i*4 + 2]);
//                value = (value-min)/(max-min)* 255.0;
//                img->setPixel(width_idx, rendered_height - height_idx -1, qRgb(value, value, value));
//                i ++;
//            }
//        }

//        *img = img->scaled(scaled_width, scaled_height);

//        int channels = 3;
//        unsigned char * sensor_data = (unsigned char *)img_ptr->data.data();
////        cv::Mat cvImage = cv::Mat(image.height(), image.width(), CV_8UC3);
//        for(int y = 0; y < scaled_height; ++y, sensor_data+=scaled_width*sizeof(float))
//        {
//            for(int x = 0; x < scaled_width; ++x)
//            {
//                QRgb rgb = img->pixel(x, y);
//                sensor_data[x * channels+2] = qRed(rgb); //r
//                sensor_data[x * channels+1] = qGreen(rgb); //g
//                sensor_data[x * channels] = qBlue(rgb); //b
//            }
//        }

//    }
//}

void DepthRenderer::renderDepthImage(sensor_msgs::Image * img_ptr)
{
    myRenderer->render(shaderSep);
    //spy->getPixelBufferImage(read);
    SbBool result;
    result = myRenderer->writeToFile("test","png");

    //shaderSep->removeAllChildren();
    //shaderSep->unref();



    unsigned char * data = myRenderer->getBuffer();
    if (verbose)
    {
      ROS_ERROR_STREAM("Test char 0:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[0]);
      ROS_ERROR_STREAM("Test char 1:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[1]);
      ROS_ERROR_STREAM("Test char 2:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[2]);
      ROS_ERROR_STREAM("Test char 3:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[3]);
      //#ifdef GRASPITDBG
      //  if (result)
      //    fprintf(stderr,"saved\n");
      //  else
      //    fprintf(stderr,"not saved\n");
      //#endif
    }

    float scale_factor = 10000.0; // output units to meters conversion factor
    //std::cout << "image_ptr: " << img_ptr << "\n";

    if(img_ptr != NULL)
    {
      img_ptr->height = myRenderer->getViewportRegion().getWindowSize()[1];
      img_ptr->width = myRenderer->getViewportRegion().getWindowSize()[0];
      img_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      img_ptr->data.resize(img_ptr->height*img_ptr->width*sizeof(float));
      img_ptr->step = img_ptr->width*sizeof(float);
      img_ptr->is_bigendian = false;
      img_ptr->header.frame_id = "/camera";
      img_ptr->header.stamp = ros::Time::now();

      float * float_data = (float*)img_ptr->data.data();

      //std::cout << " data size: " << img_ptr->data.size() << std::endl;

      for(size_t i = 0; i < img_ptr->data.size()/sizeof(float); ++ i)
      {
          float_data[i] = (data[i*4]*255*255 + data[i*4 + 1]*255 + data[i*4 + 2])/scale_factor;
          if (verbose)
              std::cout << " float_data: " << float_data[i] << std::endl;
      }
    }
}

//void DepthRenderer::renderDepthImage(sensor_msgs::Image * img_ptr)
//{

//    SoQtRenderArea *renderArea;
//    SoGLRenderAction *glRend;
//    SoOffscreenRenderer *myRenderer;
//    SoVertexShader *vertexShader = new SoVertexShader;
//    SoFragmentShader *fragmentShader = new SoFragmentShader;
//    SoShaderProgram *shaderProgram = new SoShaderProgram;
//    SoSeparator *shaderSep = new SoSeparator;
//    SoDepthBuffer * depthBuffer = new SoDepthBuffer;
//   // SoGLFrameBufferSpy *spy = new SoGLFrameBufferSpy;
//    bool verbose = false;
//    const SbColor white(0, 0, 0);

//    //    SoReadableSceneTexture2 *read = new SoReadableSceneTexture2;

////    read->type = SoReadableSceneTexture2::RGBA32F;
////    read->size.setValue(512,512);

//    //  read->type = SoSceneTexture2::RGBA32F;
//    //  read->ref();
//    //  read->scene = shaderSep;
//    //  read->model.setValue(SoSceneTexture2::NONE);
//    //  sceneTex->GLRender(glRend);

//    //I removed this
//    shaderSep->ref();
//    depthBuffer->function = SoDepthBufferElement::LESS;
//    depthBuffer->test = true;
//    depthBuffer->write = true;
//    renderArea = graspItGUI->getIVmgr()->getViewer();
//    //shaderSep->ref();
////    if(cam)
////    {
////        SoPerspectiveCamera * camera = static_cast<SoPerspectiveCamera* >(getIVmgr()->getViewer()->getCamera());
////        setCameraFromInfo(*cam, camera);

////    }



////    std::string script_path = ros::package::getPath("graspit_ql_ros") + std::string("/render_scripts");
//    std::string script_path = std::string("/home/jalapeno/curg/tactile_simulation_graspit_ws/src/graspit_pointcloud_pub/render_scripts");




//    vertexShader->sourceProgram.setValue((script_path + "/depth_vert.glsl").c_str());
//    fragmentShader->sourceProgram.setValue((script_path + "/depth_frag.glsl").c_str());


//    shaderProgram->shaderObject.set1Value(0, vertexShader);
//    shaderProgram->shaderObject.set1Value(1, fragmentShader);


//    //shaderSep->addChild(spy);
//    shaderSep->addChild(depthBuffer);
//    shaderSep->addChild(shaderProgram);
//    shaderSep->addChild(graspItGUI->getIVmgr()->getViewer()->getCamera());
//    shaderSep->addChild(renderArea->getSceneGraph());


//    //getIVmgr()->getWorld()->getIVRoot()->insertChild(shaderProgram,0);
//  glRend = new SoGLRenderAction(renderArea->getViewportRegion());
//  glRend->setSmoothing(false);
//  //  glRend->setNumPasses(5);
//  glRend->setTransparencyType(SoGLRenderAction::NONE);



//  myRenderer = new SoOffscreenRenderer(glRend);
//  myRenderer->setBackgroundColor(white);
//  #ifdef GRASPITDBG
//  if (myRenderer->isWriteSupported("jpg"))
//    std::cout << " supports jpg" << std::endl;
//  else
//    std::cout << "no jpg support" << std::endl;
//  #endif

//  myRenderer->setComponents(SoOffscreenRenderer::RGB_TRANSPARENCY);


//  myRenderer->render(shaderSep);
//  //spy->getPixelBufferImage(read);
//  SbBool result;
//  result = myRenderer->writeToFile("test","png");

//  //shaderSep->removeAllChildren();
//  //shaderSep->unref();



//  unsigned char * data = myRenderer->getBuffer();
//  if (verbose)
//  {
//    ROS_ERROR_STREAM("Test char 0:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[0]);
//    ROS_ERROR_STREAM("Test char 1:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[1]);
//    ROS_ERROR_STREAM("Test char 2:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[2]);
//    ROS_ERROR_STREAM("Test char 3:" << std::hex << std::setfill('0') << std::setw(2) << (int)data[3]);
//    //#ifdef GRASPITDBG
//    //  if (result)
//    //    fprintf(stderr,"saved\n");
//    //  else
//    //    fprintf(stderr,"not saved\n");
//    //#endif
//  }

//  float scale_factor = 10000.0; // output units to meters conversion factor
//  //std::cout << "image_ptr: " << img_ptr << "\n";

//  if(img_ptr != NULL)
//  {
//    img_ptr->height = myRenderer->getViewportRegion().getWindowSize()[1];
//    img_ptr->width = myRenderer->getViewportRegion().getWindowSize()[0];
//    img_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
//    img_ptr->data.resize(img_ptr->height*img_ptr->width*sizeof(float));
//    img_ptr->step = img_ptr->width*sizeof(float);
//    img_ptr->is_bigendian = false;
//    img_ptr->header.frame_id = "/camera";
//    img_ptr->header.stamp = ros::Time::now();

//    float * float_data = (float*)img_ptr->data.data();

//    //std::cout << " data size: " << img_ptr->data.size() << std::endl;

//    for(size_t i = 0; i < img_ptr->data.size()/sizeof(float); ++ i)
//    {
//        float_data[i] = (data[i*4]*255*255 + data[i*4 + 1]*255 + data[i*4 + 2])/scale_factor;
//        if (verbose)
//            std::cout << " float_data: " << float_data[i] << std::endl;
//    }
//  }


//    //sceneTex->scene = shaderSep;
//    //sceneTex->size.setValue(256,256);//myRenderer->getViewportRegion().getViewportSizePixels();
//    //sceneTex->type = SoSceneTexture2::RGBA32F;
//    //glRend = new SoGLRenderAction(SbViewportRegion());
//    //glRend->setSmoothing(FALSE);
//    //glRend->setNumPasses(1);
//    //glRend->apply(sceneTex);
//    //ROS_ERROR("Rendered scene\n");

//    //sceneTex->unref();
//    //depthBuffer->write
//    //shaderSep->removeChild(depthBuffer);
//    //shaderSep->unref();

//    delete myRenderer;
//}
