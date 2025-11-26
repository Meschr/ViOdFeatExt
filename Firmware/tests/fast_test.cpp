#include <opencv2/opencv.hpp>
#include <ctime>
#include "ProcessData.h"

int main() {
    std::string image_path1 = "/home/jetson/Software/ViOdFeatExt/LogData/2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm/left_000100.png";
    std::string image_path2 = "/home/jetson/Software/ViOdFeatExt/LogData/2025-11-18_11-47-30_linearMovement1TowardsSatellite500mm/right_000101.png";
    
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