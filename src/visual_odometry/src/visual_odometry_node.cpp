#include <ros/ros.h>
#include <visual_odometry/VisualOdometry.h>
#include <visual_odometry/PlottingUtils.h>

int main (int argc, char **argv) {
    ros::init(argc, argv, "visual_odometry_node");

    ros::NodeHandle nh;

    vslam::VisualOdometry* voObj = new vslam::VisualOdometry(argc, argv); 

    // vslam::getCommandLineInformation(argc, argv, voObj);

	// voObj->readImages();

	// std::cout << "The object data dir is" << voObj->dataDir << std::endl;

    plot();

    return 0;
}
