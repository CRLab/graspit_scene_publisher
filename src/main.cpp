
#include "graspit_scene_publisher_plugin.h"

extern "C" Plugin* createPlugin() {
  return new graspit_scene_publisher::GraspitScenePublisherPlugin();
}

extern "C" std::string getType() {
  return "graspit_scene_publisher_plugin";
}
