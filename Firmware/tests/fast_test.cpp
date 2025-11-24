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
    auto orb = cv::ORB::create(100);  
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
            if (m[0].distance < 0.5f * m[1].distance) { //can change this value to say how much better the best match has to be compared to second best
                good.push_back(m[0]);
            }}
    return good;
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
    std::string image_path1 = "/home/robinazv/ViOdFeatExt/LogData/2025-11-18_1/2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm/right_000001.png";
    std::string image_path2 = "/home/robinazv/ViOdFeatExt/LogData/2025-11-18_1/2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm/left_000000.png";
    cv::Mat imgL = cv::imread(image_path1, cv::IMREAD_GRAYSCALE);
    cv::Mat imgR = cv::imread(image_path2, cv::IMREAD_GRAYSCALE);
    
    clock_t start = clock();

	auto [keypoints1, descriptors1] = single_ORB(imgL);
    clock_t one_img = clock();

    auto [keypoints2, descriptors2] = single_ORB(imgR);
    clock_t two_img = clock();

    auto matches = descriptor_matcher(descriptors1, descriptors2);
    clock_t matcher = clock();


    draw_and_show(imgL, imgR, keypoints1, keypoints2, matches);

    // got the clock from https://docs.vultr.com/cpp/standard-library/ctime/clock 
    clock_t whole = clock();
    double one_img_time = double(one_img - start) / CLOCKS_PER_SEC;
    double two_img_time = double(two_img - one_img) / CLOCKS_PER_SEC;
    double matcher_time = double(matcher - two_img) / CLOCKS_PER_SEC;
    double elapsed = one_img_time + two_img_time + matcher_time;
    std::cout<<"Time for first image ORB: " << one_img_time << " seconds." << std::endl;
    std::cout<<"Time for second image ORB: " << two_img_time << " seconds." << std::endl;
    std::cout<<"Time for matching descriptors: " << matcher_time << " seconds." << std::endl;
    std::cout << "Spent time: " << elapsed << " seconds." << std::endl;

    cv::waitKey(0);
    return 0;
}