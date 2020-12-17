#include <visual_odometry/VisualOdometry.h>

DEFINE_bool(show_individual_img_features, true,
			"A boolean flag that decides whether to show features in detectORBFeatures");

namespace vslam{
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

}

 // source for reading files : https://stackoverflow.com/questions/31346132/how-to-get-all-images-in-folder-using-c
 // [DO NOT USE FILESYSTEM: NASTY GOTCHA!]
 //
void VisualOdometry::readImages() {
	std::vector<cv::String> fn;
	cv::glob(this->dataDir + "*.png", fn, false);

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

VisualOdometry::VisualOdometry(int argc, char** argv) {

	// Initiate google logging
    google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr=1;
	gflags::ParseCommandLineFlags(&argc, &argv, true);

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

