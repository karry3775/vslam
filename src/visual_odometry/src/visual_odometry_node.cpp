#include <ros/ros.h>
#include <visual_odometry/VisualOdometry.h>
#include <visual_odometry/PlottingUtils.h>

int main (int argc, char **argv) {
    ros::init(argc, argv, "visual_odometry_node");

    ros::NodeHandle nh;

    vslam::VisualOdometry* voObj = new vslam::VisualOdometry(argc, argv); 

    voObj->setImageAndCalibPaths(argc, argv, voObj);
    voObj->setCalibParams();

    // Print out if we can see the P0_
    LOG(INFO) << "The value for P0_ is: " << voObj->getP0_();
    LOG(INFO) << "The value for P1_ is: " << voObj->getP1_();
    LOG(INFO) << "The value for P2_ is: " << voObj->getP2_();
    LOG(INFO) << "The value for P3_ is: " << voObj->getP3_();
    LOG(INFO) << "The value for Tr_ is: " << voObj->getTr_();

	// voObj->readImages();


	// std::cout << "The object data dir is" << voObj->dataDir << std::endl;

    // vslam::drawFakeMap();

    return 0;
}
