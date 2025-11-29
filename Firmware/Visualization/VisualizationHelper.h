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

#endif // VISUALIZATIONHELPER_H
