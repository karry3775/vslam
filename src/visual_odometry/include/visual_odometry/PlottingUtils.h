#pragma once

#include <visual_odometry/Core.h>
#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

namespace vslam{

const int w_ = 640;
const int h_ = 480;
void drawCube();
void drawCubeOffScreen();
void drawSimpleScene();
void drawSimplePlot();
void drawFakeMap();

} // namespace vslam

