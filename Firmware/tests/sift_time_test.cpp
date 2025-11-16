#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctime>


// function to just get the keypoints and the descriptors using SIFT
std::pair<std::vector<cv::KeyPoint>, cv::Mat> single_SIFT(const cv::Mat& img) {
    auto sift = cv::SIFT::create(30);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    sift->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
    return { keypoints, descriptors };
}

// function to match the descirptors of 2 images using Brute Force matcher
std::vector<cv::DMatch> descriptor_matcher(const cv::Mat& descriptors1, const cv::Mat& descriptors2) {
    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    return matches;
}

// to display the images with the matched keypoints
void draw_and_show(const cv::Mat& imgL1,
                   const cv::Mat& imgL2,
                   const std::vector<cv::KeyPoint>& keypoints1,
                   const std::vector<cv::KeyPoint>& keypoints2,
                   const std::vector<cv::DMatch>& matches) {
    cv::Mat img_matches;
    cv::drawMatches(imgL1, keypoints1, imgL2, keypoints2, matches, img_matches);

    cv::Mat small_matches;
    cv::resize(img_matches, small_matches, cv::Size(), 0.8, 0.8);
    cv::imshow("SIFT Feature Match", small_matches);
}

int main() {
    std::string image_path1 = "left_000000.png";
    std::string image_path2 = "left_000002.png";
    cv::Mat imgL = cv::imread(image_path1, cv::IMREAD_GRAYSCALE);
    cv::Mat imgR = cv::imread(image_path2, cv::IMREAD_GRAYSCALE);

    clock_t start = clock();

	auto [keypoints1, descriptors1] = single_SIFT(imgL);
	auto [keypoints2, descriptors2] = single_SIFT(imgR);

    auto matches = descriptor_matcher(descriptors1, descriptors2);

	draw_and_show(imgL, imgR, keypoints1, keypoints2, matches);



    // got the clock from https://docs.vultr.com/cpp/standard-library/ctime/clock 
    clock_t end = clock();
    double elapsed = double(end - start) / CLOCKS_PER_SEC;
    std::cout << "Spent time: " << elapsed << " seconds." << std::endl;


    cv::waitKey(0);
    return 0;
}