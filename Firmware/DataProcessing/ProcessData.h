#include <opencv2/opencv.hpp>

struct StereoLandmark
{
  cv::Point3f position;
  cv::Mat descriptor;
};

std::vector<cv::KeyPoint> single_FAST(const cv::Mat &img);
std::pair<std::vector<cv::KeyPoint>, cv::Mat> single_ORB(const cv::Mat &img);
std::pair<std::vector<cv::KeyPoint>, cv::Mat> single_SIFT(const cv::Mat &img);

std::vector<cv::DMatch> descriptor_matcher(const cv::Mat &descriptors1,
                                           const cv::Mat &descriptors2,
                                           const float thresh = 0.5f);
void draw_and_show(const cv::Mat &imgL,
                   const cv::Mat &imgR,
                   const std::vector<cv::KeyPoint> &keypoints1,
                   const std::vector<cv::KeyPoint> &keypoints2,
                   const std::vector<cv::DMatch> &matches);

std::vector<cv::Point3f> stereo_3Dpoints(const cv::Mat &P1,
                                         const cv::Mat &P2,
                                         const std::vector<cv::KeyPoint> &keypoints1,
                                         const std::vector<cv::KeyPoint> &keypoints2,
                                         const std::vector<cv::DMatch> &matches);