#ifndef SOREADABLESCENETEXTURE2_H_
#define SOREADABLESCENETEXTURE2_H_

#include <Inventor/nodes/SoSceneTexture2.h>
#include <Inventor/C/glue/gl.h>

class SoReadableSceneTexture2 : public SoSceneTexture2
{
        SO_NODE_HEADER(SoReadableSceneTexture2);
private:
        static bool initialized;
public:
        static void initClass(void);
        SoReadableSceneTexture2(void);
        virtual void GLRender(SoGLRenderAction * action);
        const cc_glglue * glue;

};

#endif