#include "VisualizationHelper.h"
#include <iostream>

using namespace cv;
using namespace std;

int main() {
  // Example 3D keypoints (point cloud data)
  std::vector<cv::Point3f> pointCloud = {
      {0.0f, 0.0f, 0.0f},    // Origin
      {1.0f, 0.0f, 0.0f},    // X-axis
      {0.0f, 1.0f, 0.0f},    // Y-axis
      {0.0f, 0.0f, 1.0f},    // Z-axis
      {1.0f, 1.0f, 1.0f},    // XYZ diagonal
      {-1.0f, -1.0f, -1.0f}, // Opposite corner
      {2.0f, 0.5f, -0.5f},   // Arbitrary point
  };

  // Example labels for each point
  std::vector<std::string> labels = {
      "Origin (0, 0, 0)",        "X-Axis (1, 0, 0)",   "Y-Axis (0, 1, 0)",
      "Z-Axis (0, 0, 1)",        "Diagonal (1, 1, 1)", "Corner (-1, -1, -1)",
      "Arbitrary (2, 0.5, -0.5)"};

  // Visualize the 3D keypoints with tooltips
  visualize3DKeypointsWithTooltips(pointCloud, labels);
  return 0;
}
