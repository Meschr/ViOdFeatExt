cmake_minimum_required(VERSION 3.10)
project(firmware_tests_examples LANGUAGES C CXX)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(checkboard_calibration ${CMAKE_CURRENT_SOURCE_DIR}/checkboard_calibration.cpp)# add_executable(approx_calibration ${CMAKE_CURRENT_SOURCE_DIR}/approximationCalibration.cpp)
add_executable(test_depth ${CMAKE_CURRENT_SOURCE_DIR}/depth_test.cpp)

# Ensure headers are findable
target_include_directories(checkboard_calibration PRIVATE
  {CMAKE_SOURCE_DIR}/../DataProcessing
)

# Ensure headers are findable
# target_include_directories(approx_calibration PRIVATE
#   {CMAKE_SOURCE_DIR}/../DataProcessing
# )

# Ensure headers are findable
target_include_directories(test_depth PRIVATE
  {CMAKE_SOURCE_DIR}/../Capture
  {CMAKE_SOURCE_DIR}/../DataProcessing
)

# Link the firmware libraries built by parent
target_link_libraries(checkboard_calibration PRIVATE dataProcessing)
# target_link_libraries(approx_calibration PRIVATE dataProcessing)
target_link_libraries(test_depth PRIVATE captureDevice dataProcessing)

# Optionally set C++ standard
target_compile_features(checkboard_calibration PRIVATE cxx_std_11)
# target_compile_features(approx_calibration PRIVATE cxx_std_11)
target_compile_features(test_depth PRIVATE cxx_std_11)
