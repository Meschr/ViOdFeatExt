#ifndef VISUALIZATIONHELPER_H
#define VISUALIZATIONHELPER_H

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <vector>

// Visualizes 3D keypoints
void visualize3DKeypoints(const std::vector<cv::Point3f> &positions);

// Visualizes 3D keypoints with tooltips
void visualize3DKeypointsWithTooltips(const std::vector<cv::Point3f> &positions,
                                      const std::vector<std::string> &labels);
// Visualizes a trajectory in 3D space
void visualizeTrajectory(const std::vector<cv::Point3f> &trajectory,
                         const cv::viz::Color &color = cv::viz::Color::blue());
#endif // VISUALIZATIONHELPER_H
