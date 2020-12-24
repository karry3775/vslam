#include <visual_odometry/VisualOdometry.h>

DEFINE_bool(show_individual_img_features, true,
			"A boolean flag that decides whether to show features in detectORBFeatures");

namespace vslam{

VisualOdometry::VisualOdometry(int argc, char** argv) : image_dir_{""}, calib_path_{""} {

	// Initiate google logging
    google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr=1;
	gflags::ParseCommandLineFlags(&argc, &argv, true);
}

// source for reading files : https://stackoverflow.com/questions/31346132/how-to-get-all-images-in-folder-using-c
 // [DO NOT USE FILESYSTEM: NASTY GOTCHA!]
 //
void VisualOdometry::readImages() {
	std::vector<cv::String> fn;
	cv::glob(this->image_dir_ + "*.png", fn, false);

	size_t count = fn.size();


	cv::Mat prev_img = cv::imread(fn[0]);

	for(size_t i=1; i < count; i++) {

		cv::Mat cur_img = cv::imread(fn[i]);
		if(cur_img.empty()) {
			LOG(INFO) << "File not found at : "<< fn[i] << std::endl;
		}		
		else{
			cv::imshow("Original", cur_img);
			// sanity check for matchORBFeatures
			this->matchORBFeatures(prev_img, cur_img);
			int key = cv::waitKey(1);	
			if(key == 27) {
				// pushed the escape key
				LOG(INFO) << "Quitting on Escape key!" << std::endl;
				break;
			}
		}

		prev_img = cur_img;
		 
	}

 
}

// code for orb feature detection
// sources : https://stackoverflow.com/questions/46199558/orb-feature-matching-with-flann-in-c
//    	   : https://stackoverflow.com/questions/56241536/opencv-fastbrief-how-to-draw-keypoints-with-drawmatchesflagsdraw-rich-keyp

// This function takes in a single image and overlays it with orb features
std::pair<KeyPointsVector, cv::Mat> VisualOdometry::detectORBFeatures(const std::string& frame, const cv::Mat& img) {
	
	// create the descriptors and keypoints
	KeyPointsVector keypoints;
	cv::Mat descriptor;

	cv::Ptr<cv::ORB> detector = cv::ORB::create();
	std::vector<cv::DMatch> matches;
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

	// detector->detectAndCompute(img, keypoints, descriptor);
	detector->detect(img, keypoints);
	cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
	extractor->compute(img, keypoints, descriptor);
	
	if(FLAGS_show_individual_img_features) {
		// Draw ORB Keypoints
		cv::Mat output;
		cv::drawKeypoints(img, keypoints, output, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::imshow(frame, output);
	}

	std::pair<KeyPointsVector, cv::Mat> keypoint_descriptor_pair = std::make_pair(keypoints, descriptor);

	return keypoint_descriptor_pair;
}

void VisualOdometry::matchORBFeatures(const cv::Mat& img1, const cv::Mat& img2) {

	std::pair<KeyPointsVector, cv::Mat> keypoint_descriptor_pair1 = this->detectORBFeatures("prev_frame",img1);
	std::pair<KeyPointsVector, cv::Mat> keypoint_descriptor_pair2 = this->detectORBFeatures("cur_frame",img2);

	KeyPointsVector keypoints1 = keypoint_descriptor_pair1.first;
	cv::Mat descriptors1 = keypoint_descriptor_pair1.second;
	KeyPointsVector keypoints2 = keypoint_descriptor_pair2.first;
	cv::Mat descriptors2 = keypoint_descriptor_pair2.second;


	// matching descriptors
	cv::BFMatcher matcher(cv::NORM_HAMMING, true); // changed from cv::NORM_L2, adding the crossCheck boolean
	std::vector<cv::DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);

	// Apply the sift ratio test
	std::vector<cv::DMatch> good_matches;
	for(auto match : matches) {
		if (matches[match.queryIdx].distance < 0.75 * matches[match.trainIdx].distance) {
			good_matches.push_back(match);
		}
	 }
	 
	// draw the results
	cv::namedWindow("matches", 1);
	cv::Mat img_matches;
	cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches
					, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>() , cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	cv::imshow("matches", img_matches);

	cv::namedWindow("good_matches", 1);
	cv::drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_matches
					, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>() , cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	cv::imshow("good_matches", img_matches);

	/** Experimental code **/
	// TODO: add code to actually estimate motion,
	// will make this into a function later on
	/** end of experimental code**/

}

// Setters
 void VisualOdometry::setImageDir(const std::string& image_dir) {
	 image_dir_ = image_dir;
 }

 void VisualOdometry::setCalibPath(const std::string& calib_dir) {
	 calib_path_ = calib_dir;
 }

 // Helper functions for setCalibParams (may be put this in a utils thing or something
 // to keep code clean)
Matrix34d assignVectorToMatrix(const std::vector<double>& vect) {
	Matrix34d T;
	T << float(vect[0]), float(vect[1]), float(vect[2]),  float(vect[3]),
		 float(vect[4]), float(vect[5]), float(vect[6]),  float(vect[7]),
		 float(vect[8]), float(vect[9]), float(vect[10]), float(vect[11]);
	return T;
}

 CalibParam assignCalibParam(std::string token) {
	CHECK(token == "P0:" || token == "P1:" ||
		  token == "P2:" || token == "P3:" ||
		  token == "Tr:") << "The calibration file does not have the right format to parse it!";

	if(token == "P0:"){
		return P0;
	}
	else if(token == "P1:"){
		return P1;
	}
	else if(token == "P2:"){
		return P2;
	}
	else if(token == "P3:"){
		return P3;
	}
	
	return Tr; // this would be returned if none of the conditions are satisfied
 }

 void VisualOdometry::setCalibParams() {
	// Check if the CalibDir is set
	CHECK_NE(calib_path_ , std::string("")) 
			<< "The Calibration directory is not set! Cannot access calib params!";
	std::fstream file;
	file.open(calib_path_, std::ios::in);

	if(file.is_open()) {
		std::string text;
		while(getline(file, text)) {
			size_t pos = 0;
			std::string token;
			std::string delim = " ";
			std::vector<double> calib_vector{};

			CalibParam calib_param = NOTSET;

			while ((pos = text.find(delim)) != std::string::npos) {
				token = text.substr(0, pos);
				// convert token to double
				// double token_d = atof(token.c_str());
				std::istringstream os(token);
				double token_d;
				os >> token_d;
				text.erase(0, pos + delim.length());

				// Assign which calib_param this is so that we can put this value in the right
				if(calib_param == NOTSET){
					calib_param = assignCalibParam(token);
					continue;
				}	
							
				calib_vector.push_back(token_d);
			}

			switch(calib_param) {
				case P0:
					P0_ = assignVectorToMatrix(calib_vector);
					break;
				case P1:
					P1_ = assignVectorToMatrix(calib_vector);
					break;
				case P2:
					P2_ = assignVectorToMatrix(calib_vector);
					break;
				case P3:
					P3_ = assignVectorToMatrix(calib_vector);
					break;
				case Tr:
					Tr_ = assignVectorToMatrix(calib_vector);

			}
		}
	}
	else {
		LOG(INFO) << "Cannot open the calibration file! Calib params not set!";
	}

 }


void VisualOdometry::setImageAndCalibPaths (int argc, char** argv, VisualOdometry* voObj){

    // create a boost program_options namespace
    namespace po = boost::program_options;

    // create a variables_map
    po::variables_map vm;

    // create options_description
    po::options_description desc("Allowed options!");
	
	// Default paths
	std::string image_dir = "/home/kartik/Documents/vslam_data/data_odometry_gray/dataset/sequences/00/image_0/";
	std::string calib_path = "/home/kartik/Documents/vslam_data/data_odometry_calib/dataset/sequences/00/calib.txt";
    // add options
    desc.add_options()
    ("image_dir", po::value<std::string>()->default_value(image_dir), 
    "Directory for kitti dataset containing image sequences!")
	("calib_path", po::value<std::string>()->default_value(calib_path),
	"Path for the calibration parameters text file!");

    // link vma and options
    po::store(po::parse_command_line(argc, argv, desc), vm);

    try{
        po::notify(vm);
    } catch (std::exception& e){
        LOG(INFO) << "Error: " << e.what() << std::endl;
        LOG(INFO) << desc << std::endl;
        LOG(FATAL) << "Exiting!";
    }   

	voObj->setImageDir(vm["image_dir"].as<std::string>());
	voObj->setCalibPath(vm["calib_path"].as<std::string>());
}

// Getters
std::string VisualOdometry::getImageDir() const{
	return image_dir_;
}

std::string VisualOdometry::getCalibPath() const{
	return calib_path_;
}

Matrix34d VisualOdometry::getP0_() const {
	return P0_;
}

Matrix34d VisualOdometry::getP1_() const {
	return P1_;
}

Matrix34d VisualOdometry::getP2_() const {
	return P2_;
}

Matrix34d VisualOdometry::getP3_() const {
	return P3_;
}

Matrix34d VisualOdometry::getTr_() const {
	return Tr_;
}
     

} // namespace vslam

