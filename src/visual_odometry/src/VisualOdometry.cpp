#include <visual_odometry/VisualOdometry.h>
#include <experimental/filesystem> // to read files
#include <exception>
#include <sstream>
#include <vector>

namespace vslam{


 // source for reading files : https://stackoverflow.com/questions/31346132/how-to-get-all-images-in-folder-using-c
 // [DO NOT USE FILESYSTEM: NASTY GOTCHA!]
 //
 void VisualOdometry::readImages() {
	std::vector<cv::String> fn;
	cv::glob(this->dataDir + "*.png", fn, false);

	size_t count = fn.size();

	for(size_t i=0; i < count; i++) {

		cv::Mat img = cv::imread(fn[i]);
		if(img.empty()) {
			LOG(INFO) << "File not found at : "<< fn[i] << std::endl;
		}		
		else{
			cv::imshow("Display", img);
		}
		cv::waitKey(1);
	}

 
}

VisualOdometry::VisualOdometry(char** argv) {

	// Initiate google logging
	
    google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr=1;

}

void getCommandLineInformation(int argc, char** argv, VisualOdometry* voObj){

    // create a boost program_options namespace
    namespace po = boost::program_options;

    // create a variables_map
    po::variables_map vm;

    // create options_description
    po::options_description desc("Allowed options!");

	// DEBUG ERROR
	std::string prev_path = "/home/kartik/Documents/vslam_data/data_odometry_gray/dataset/sequences/00/image_0/";
	std::string new_path = "/home/kartik/Downloads/images/";
    // add options
    desc.add_options()
    ("data_dir", po::value<std::string>()->default_value(prev_path), 
    "Path for kitti data!");

    // link vma and options
    po::store(po::parse_command_line(argc, argv, desc), vm);

    try{
        po::notify(vm);
    } catch (std::exception& e){
        LOG(INFO) << "Error: " << e.what() << std::endl;
        LOG(INFO) << desc << std::endl;
        LOG(FATAL) << "Exiting!";
    }   

    voObj->dataDir = vm["data_dir"].as<std::string>();

    LOG(INFO) << "Opening file : " << voObj->dataDir << std::endl;
}
     

} // namespace vslam

