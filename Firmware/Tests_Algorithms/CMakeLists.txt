cmake_minimum_required(VERSION 3.10)
project(algorithm_tests LANGUAGES C CXX)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)

add_executable(test_sift_time ${CMAKE_CURRENT_SOURCE_DIR}/sift_time_test.cpp)
add_executable(test_fast_time ${CMAKE_CURRENT_SOURCE_DIR}/fast_test.cpp)
add_executable(test_kp_tracking ${CMAKE_CURRENT_SOURCE_DIR}/keypoint_tracker_test.cpp)
add_executable(test_ekf ${CMAKE_CURRENT_SOURCE_DIR}/imu_vo_ekf.cpp)

# fixing opencv to work for me
target_include_directories(test_sift_time PRIVATE 
  ${CMAKE_SOURCE_DIR}/../DataProcessing
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(test_fast_time PRIVATE 
  {CMAKE_SOURCE_DIR}/../DataProcessing
  {OpenCV_INCLUDE_DIRS}
)

target_include_directories(test_kp_tracking PRIVATE 
  ${CMAKE_SOURCE_DIR}/../DataProcessing
  ${CMAKE_SOURCE_DIR}/../Capture
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../Utility
)

target_include_directories(test_ekf PRIVATE 
  ${CMAKE_SOURCE_DIR}/../DataProcessing
  ${OpenCV_INCLUDE_DIRS}
)

# Link the firmware libraries built by parent
target_link_libraries(test_sift_time PRIVATE ${OpenCV_LIBS} dataProcessing)
target_link_libraries(test_fast_time PRIVATE ${OpenCV_LIBS} dataProcessing)
target_link_libraries(test_kp_tracking PRIVATE ${OpenCV_LIBS} dataProcessing captureDevice Utility)
target_link_libraries(test_ekf PRIVATE ${OpenCV_LIBS} ekfProcessing Eigen3::Eigen)

# Optionally set C++ standard
target_compile_features(test_sift_time PRIVATE cxx_std_11)
target_compile_features(test_fast_time PRIVATE cxx_std_11)
target_compile_features(test_kp_tracking PRIVATE cxx_std_11)
target_compile_features(test_ekf PRIVATE cxx_std_11)