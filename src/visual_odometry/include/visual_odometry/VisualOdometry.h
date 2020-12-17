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
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>

namespace vslam{


class VisualOdometry{

private:
	const int max_features_ = 500;
public:
  // Default constructor
	VisualOdometry();
	VisualOdometry(char** argv); 
	void readImages();
    void detectORBFeatures(cv::Mat& img);		  					
	std::string dataDir;
	
};

void getCommandLineInformation(int argc, char** argv, VisualOdometry* voObj);

} // namespace vslam




#endif
