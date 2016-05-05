#include <SoReadableSceneTexture2.h>
#include <Inventor/C/glue/gl.h>
#include <GL/gl.h>

class SoGLFrameBufferSpy : public SoNode
{
        SO_NODE_HEADER(SoGLFrameBufferSpy);
private:
        static bool initialized;
public:
        static void initClass(void);
        SoGLFrameBufferSpy(void);
        virtual void GLRender(SoGLRenderAction * action);

        void getPixelBufferImage(SoReadableSceneTexture2 *scene);
protected:
        GLint fbo_framebuffer_;
        GLint gl_drawbuffer_mode_;
        GLint texture_id_;
        GLint texture_level_;
};
