#include "VisualizationHelper.h"
#include <iostream>

void visualize3DKeypoints(const std::vector<cv::Point3f> &positions) {
  // Check for empty positions
  if (positions.empty()) {
    std::cerr << "Error: No 3D positions to visualize!" << std::endl;
    return;
  }

  // Create a 3D visualizer
  cv::viz::Viz3d visualizer("3D Keypoint Visualization");

  // Create point cloud widget and add points
  cv::viz::WCloud cloud(positions, cv::viz::Color::green());
  visualizer.showWidget("Keypoint Cloud", cloud);

  // Start the interactive visualizer
  std::cout << "Press 'q' to exit the visualization window." << std::endl;
  visualizer.spin();
}

// Function to visualize 3D keypoints with tooltips
void visualize3DKeypointsWithTooltips(const std::vector<cv::Point3f> &positions,
                                      const std::vector<std::string> &labels) {
  // Check if positions are empty or if sizes of positions and labels mismatch
  if (positions.empty()) {
    std::cerr << "Error: No 3D positions to visualize!" << std::endl;
    return;
  }

  if (positions.size() != labels.size()) {
    std::cerr << "Error: Size of positions and labels must match!" << std::endl;
    return;
  }

  // Create a 3D visualizer
  cv::viz::Viz3d visualizer("3D Keypoint Visualization with Tooltips");

  // Create a green point cloud
  cv::viz::WCloud cloud(positions, cv::viz::Color::green());
  visualizer.showWidget("Keypoint Cloud", cloud);

  // Add tooltip annotations as text for each 3D point
  for (size_t i = 0; i < positions.size(); ++i) {
    // Add a 3D text description next to the point
    cv::viz::WText3D text(
        labels[i], // Label text
        positions[i] +
            cv::Point3f(0.1f, 0.1f, 0.1f), // Slightly offset from the point
        0.05,                              // Font size
        true,                              // Face-to-camera flag
        cv::viz::Color::yellow()           // Text color
    );

    // Add the text to the visualizer
    visualizer.showWidget("Tooltip_" + std::to_string(i), text);
  }

  // Start the interactive visualizer
  std::cout << "Press 'q' to exit the visualization window." << std::endl;
  visualizer.spin();
}
