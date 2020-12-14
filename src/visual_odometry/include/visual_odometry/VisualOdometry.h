#ifndef VISUAL_ODOMETRY_FILE_H_
#define VISUAL_ODOMETRY_FILE_H_

// ros related headers
#include <ros/ros.h>

// general
#include <boost/program_options.hpp>
#include <glog/logging.h>
#include <string>

// Opencv related headers
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

namespace vslam{


class VisualOdometry{

public:
  // Default constructor
  VisualOdometry();
  VisualOdometry(char** argv); 
  void readImages();
  std::string dataDir;
};

void getCommandLineInformation(int argc, char** argv, VisualOdometry* voObj);

} // namespace vslam




#endif
