
#include <SoReadableSceneTexture2.h>
#include <Inventor/nodes/SoSubNode.h>
#include <Inventor/elements/SoElements.h>
#include <Inventor/elements/SoGLMultiTextureImageElement.h>
#include <Inventor/elements/SoGLMultiTextureEnabledElement.h>
#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/C/glue/gl.h>
#include <iostream>
#include <Inventor/misc/SoGLImage.h>
#include <Inventor/elements/SoTextureUnitElement.h>
#include <Inventor/elements/SoTextureImageElement.h>

SO_NODE_SOURCE(SoReadableSceneTexture2);


bool SoReadableSceneTexture2::initialized;

// Documented in superclass.
void
SoReadableSceneTexture2::initClass(void)
{
        SO_NODE_INIT_CLASS(SoReadableSceneTexture2, SoSceneTexture2, "nodes");

        SO_ENABLE(SoGLRenderAction, SoGLMultiTextureImageElement);
        SO_ENABLE(SoGLRenderAction, SoGLMultiTextureEnabledElement);

        SoReadableSceneTexture2::initialized = true;
}



SoReadableSceneTexture2::SoReadableSceneTexture2():SoSceneTexture2()
{
        if(!SoReadableSceneTexture2::initialized)
                this->initClass();
}


void SoReadableSceneTexture2::GLRender(SoGLRenderAction * action)
{
    SoState * state = action->getState();

    SoTextureImageElement * elem = (SoTextureImageElement*)state->getConstElement(SoTextureImageElement::getClassStackIndex());
    //std::cout << "Element size: "<< elem->size[0] << " , num_components: " << num_components << std::endl;
    int unit = SoTextureUnitElement::get(state);
    SbVec2s sizev = size.getValue();
    int num_components(4);
    std::cout << "Texture unit: " <<unit <<" size: " << sizev[0] <<", " << sizev[1] << " , num_components: " << num_components << std::endl;
    SoGLMultiTextureImageElement::getImage(state, unit, sizev, num_components);

    std::cout << "Texture unit: " <<unit <<" size: " << sizev[0] <<", " << sizev[1] << " , num_components: " << num_components << std::endl;
    int32_t cachecontext = SoGLCacheContextElement::get(state);
    glue = cc_glglue_instance(cachecontext);
    SoSceneTexture2::GLRender(action);
    state = action->getState();
    //GLuint fbo_frameBuffer = glGetIntegerv(FRAMEBUFFER_BINDING_EXT);
    //cc_glglue_glBindFramebuffer(glue, GL_FRAMEBUFFER_EXT, fbo_frameBuffer);
    std::cout << "Rendered something!\n";
    //SoGLImage * glimage = SoGLMultiTextureImageElement::get(state, unit, glmodel,this->blendColor.getValue());
    //glimage->getGLDisplayList()->
    unit = SoTextureUnitElement::get(state);
    SoGLMultiTextureImageElement::getImage(state, unit, sizev, num_components);
    std::cout << "Texture unit: " <<unit <<" size: " << sizev[0] <<", " << sizev[1] << " , num_components: " << num_components << std::endl;


}
