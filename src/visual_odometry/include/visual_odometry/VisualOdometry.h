#ifndef VISUAL_ODOMETRY_FILE_H_
#define VISUAL_ODOMETRY_FILE_H_

// ros related headers
#include <ros/ros.h>

// general
#include <boost/program_options.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <string>
#include <exception>
#include <sstream>
#include <vector>

// Opencv related headers
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp> // for drawing circles, lines etc.

// some typdefs
typedef std::vector<cv::KeyPoint> KeyPointsVector;

namespace vslam{


class VisualOdometry{

private:
	const int max_features_ = 500;
public:
  // Default constructor
	VisualOdometry();
	VisualOdometry(int argc, char** argv); 
	void readImages();
    std::pair<KeyPointsVector, cv::Mat> detectORBFeatures(const std::string&, const cv::Mat&);
	void matchORBFeatures(const cv::Mat&, const cv::Mat&);

	std::string dataDir;
	
};

void getCommandLineInformation(int argc, char** argv, VisualOdometry* voObj);

} // namespace vslam




#endif
