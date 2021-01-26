#ifndef VISUAL_ODOMETRY_FILE_H_
#define VISUAL_ODOMETRY_FILE_H_

// Core
#include <visual_odometry/Core.h>

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

// Eigen 
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// For file handling
#include <fstream>


// some typdefs
typedef std::vector<cv::KeyPoint> KeyPointsVector;
typedef Eigen::Matrix<double, 3, 4> Matrix34d;

namespace vslam{

enum CalibParam {P0, P1, P2, P3, Tr, NOTSET};

class VisualOdometry{

private:
	std::string image_dir_; // directory containing image sequences
	std::string calib_path_; // path to calibration data
	
	Matrix34d P0_ ; // TODO define what this is
	Matrix34d P1_; // TODO define what this is
	Matrix34d P2_; // TODO define what this is
	Matrix34d P3_; // TODO define what this is
	Matrix34d Tr_; // TODO define what this is

public:  	
	VisualOdometry();// Default constructor
	VisualOdometry(int argc, char** argv); 
	void readImages();
    std::pair<KeyPointsVector, cv::Mat> detectORBFeatures(const std::string&, const cv::Mat&);
	void matchORBFeatures(const cv::Mat&, const cv::Mat&);

	// Setters
	/** 
	 * \brief: Sets the directory containing image sequences
	 */ 
	void setImageDir(const std::string&);

	/**
	 * \brief : Sets the file path for calibration data
	 */ 
	void setCalibPath(const std::string&);

	/**
	 *  \brief: Set values for P0_, P1_, P2_, P3_, Tr_
	 * 
	 *  Uses the calib_dir_ which must have been previously set by setCalibDir
	 */  
	void setCalibParams(); 

	/** 
	 * \brief : Sets both the image and calibration path
	 */
	void setImageAndCalibPaths(int, char**, VisualOdometry*);

	// Getters
	/**
	 * \brief : Returns the image_dir_
	 * @return: path to the image directory containing image sequences
	 */ 
	std::string getImageDir() const; 

	/**
	 * \brief: Return the calib_path_
	 * @return: path to the calibration file containing data for
	 * P0_, P1_, P2_, P3_, Tr_
	 */
	std::string getCalibPath() const;

	/**
	 * \brief : Returns the P0_
	 * @return : TODO - NEEDS TO BE SET and rename the getter name as you know more
	 */
	Matrix34d getP0_() const;

	/**
	 * \brief : Returns the P1_
	 * @return: TODO - NEEDS TO BE SET and rename the getter name as you know more
	 */
	Matrix34d getP1_() const;

	/**
	 * \brief : Returns the P2_
	 * @return : TODO - NEEDS TO BE SET and rename the getter name as you know more
	 */
	Matrix34d getP2_() const;

	/** \brief:  Returns the P3_
	 * @return : TODO - NEEDS TO BE SET and rename the getter name as you know more
	 */
	Matrix34d getP3_() const;

	/** \brief : Returns the Tr_
	 * @return : TODO - NEEDS TO BE SET and rename the getter name as you know more
	 */
	Matrix34d getTr_() const;  
};

void getCommandLineInformation(int argc, char** argv, VisualOdometry* voObj);

} // namespace vslam




#endif
