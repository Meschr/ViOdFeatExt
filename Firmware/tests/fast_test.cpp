#include <opencv2/opencv.hpp>
#include <ctime>

//implement the FAST for only keypoints
std::vector<cv::KeyPoint> single_FAST(const cv::Mat& img) {
    auto fast = cv::FastFeatureDetector::create(30); 
    std::vector<cv::KeyPoint> keypoints;
    fast->detect(img, keypoints);
    return keypoints;
}




//implementing ORB for finding both keypoints and their descriptors
std::pair<std::vector<cv::KeyPoint>, cv::Mat> single_ORB(const cv::Mat& img) {
    auto orb = cv::ORB::create(200);  
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
    return { keypoints, descriptors };
}





//to match the keypoints between different frames
std::vector<cv::DMatch> descriptor_matcher(const cv::Mat& descriptors1, const cv::Mat& descriptors2) {
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knnMatches;

        matcher.knnMatch(descriptors1, descriptors2, knnMatches, 2);

        std::vector<cv::DMatch> good;
        for (auto& m : knnMatches) {
            if (m[0].distance < 0.50f * m[1].distance) { //can change this value to say how much better the best match has to be compared to second best
                good.push_back(m[0]);
            }}
    return good;
}






//implementing orb and keypoint matching in one function
std::tuple<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>, std::vector<cv::DMatch>> ORB_and_match(const cv::Mat& img1, const cv::Mat& img2) {
    auto orb = cv::ORB::create(100);

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;

    orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
    orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

    auto matches = descriptor_matcher(descriptors1, descriptors2);

    return { keypoints1, keypoints2, matches };
}






// to display the images with the matched keypoints
void draw_and_show(const cv::Mat& imgL,
                   const cv::Mat& imgR,
                   const std::vector<cv::KeyPoint>& keypoints1,
                   const std::vector<cv::KeyPoint>& keypoints2,
                   const std::vector<cv::DMatch>& matches) {
    cv::Mat img_matches;
    cv::drawMatches(imgL, keypoints1, imgR, keypoints2, matches, img_matches);

    cv::Mat small_matches;
    cv::resize(img_matches, small_matches, cv::Size(), 1.5, 1.5);
    cv::imshow("ORB Feature Match", small_matches);
}



int main() {
    std::string image_path1 = "/home/robinazv/ViOdFeatExt/LogData/2025-11-18_1/2025-11-18_12-08-38_linearMovementPointedTowardsMoon300mm/left_000022.png";
    std::string image_path2 = "/home/robinazv/ViOdFeatExt/LogData/2025-11-18_1/2025-11-18_12-08-38_linearMovementPointedTowardsMoon300mm/left_000024.png";
    
    cv::Mat imgL = cv::imread(image_path1, cv::IMREAD_GRAYSCALE);
    cv::Mat imgR = cv::imread(image_path2, cv::IMREAD_GRAYSCALE);
    clock_t start = clock();


    //one function to do ORB and matching
    auto [keypoints1, keypoints2, matches] = ORB_and_match(imgL, imgR);
    clock_t t1 = clock();

    std::cout << "this is inside the keypoints: ">> keypoints1[1] << std::endl;

    //3 functions to do the ORB and matching
	//auto [keypoints1, descriptors1] = single_ORB(imgL);
    //auto [keypoints2, descriptors2] = single_ORB(imgR);
    //auto matches = descriptor_matcher(descriptors1, descriptors2);  
    clock_t t2 = clock();

    double elapsed = double(t1 - start) / CLOCKS_PER_SEC;
    double elapsed2 = double(t2 - t1) / CLOCKS_PER_SEC;
    std::cout << "Spent time for 1 function: " << elapsed << " seconds." << std::endl;
    std::cout << "Spent time for 3 functions: " << elapsed2 << " seconds." << std::endl;



    draw_and_show(imgL, imgR, keypoints1, keypoints2, matches);

    cv::waitKey(0);
    return 0;
}