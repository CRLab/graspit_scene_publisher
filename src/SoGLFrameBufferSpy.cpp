
#include <SoGLFrameBufferSpy.h>
#include <Inventor/actions/SoGLRenderAction.h>
#include <iostream>
#include <SoReadableSceneTexture2.h>
#include <QImage>
#include <GL/gl.h>
#include <Inventor/C/glue/gl.h>
#include <GL/glu.h>


SO_NODE_SOURCE(SoGLFrameBufferSpy);


bool SoGLFrameBufferSpy::initialized;

// Documented in superclass.
void
SoGLFrameBufferSpy::initClass(void)
{
        SO_NODE_INIT_CLASS(SoGLFrameBufferSpy, SoNode, "nodes");

        SoGLFrameBufferSpy::initialized = true;
}



SoGLFrameBufferSpy::SoGLFrameBufferSpy():SoNode()
{
        if(!SoGLFrameBufferSpy::initialized)
                this->initClass();
}


void SoGLFrameBufferSpy::GLRender(SoGLRenderAction * action)
{
        SoState * state = action->getState();
        //int32_t cachecontext = SoGLCacheContextElement::get(state);
        //const cc_glglue * glue = cc_glglue_instance(cachecontext);        	
        // update GL draw buffer and read buffers
        glGetIntegerv(GL_FRAMEBUFFER_BINDING, &fbo_framebuffer_);
        glGetIntegerv(GL_DRAW_BUFFER, &gl_drawbuffer_mode_);
        GLint type;
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &texture_id_);
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_LEVEL, &texture_level_);

        std::cout <<"GLError: " <<  gluErrorString(glGetError()) << "\n";
        std::cout <<"GL attached object ID is: " << texture_id_ << "\n";
}


void SoGLFrameBufferSpy::getPixelBufferImage(SoReadableSceneTexture2 * scene)
{    

return;
    GLenum status = cc_glglue_glCheckFramebufferStatus(scene->glue,GL_FRAMEBUFFER_EXT);
    std::cout <<"GL status is read before bind: " << status << "\n";

    cc_glglue_glBindFramebuffer(scene->glue, GL_FRAMEBUFFER_EXT, fbo_framebuffer_);
    std::cout <<"GLError after bind: " <<  gluErrorString(glGetError()) << "\n";
    cc_glglue_glFramebufferTexture2D(scene->glue, GL_FRAMEBUFFER_EXT,  GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, texture_id_, texture_level_);
    std::cout <<"GLError after texture bind: " <<  gluErrorString(glGetError()) << "\n";

    status = cc_glglue_glCheckFramebufferStatus(scene->glue,GL_FRAMEBUFFER_EXT);
    std::cout <<"GL status is read after bind: " << status << "\n";

    std::cout <<"GLError: " <<  gluErrorString(glGetError()) << "\n";

    GLenum format = GL_RGBA;
    GLenum type = GL_RGBA32F_ARB;
    int numLayers = 4;
    unsigned int pixelSize = numLayers*sizeof(float);
    size_t numBytes = scene->size.getValue()[0] * scene->size.getValue()[1];
    char pixels[numBytes];
    glReadPixels(0,0,scene->size.getValue()[0], scene->size.getValue()[1], format, type, pixels);

    float * pixels_as_float = (float*)(&pixels[0]);
    std::cout <<"GLError: " <<  gluErrorString(glGetError()) << "\n";
    std::cout <<"Pixels as float" << pixels_as_float[0] <<"\n";

}