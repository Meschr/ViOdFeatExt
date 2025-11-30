cmake_minimum_required(VERSION 3.10)
project(algorithm_tests LANGUAGES C CXX)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(test_sift_time ${CMAKE_CURRENT_SOURCE_DIR}/sift_time_test.cpp)
add_executable(test_fast_time ${CMAKE_CURRENT_SOURCE_DIR}/fast_test.cpp)

# fixing opencv to work for me
target_include_directories(test_sift_time PRIVATE 
  {CMAKE_SOURCE_DIR}/../DataProcessing
  {OpenCV_INCLUDE_DIRS}
)

target_include_directories(test_fast_time PRIVATE 
  {CMAKE_SOURCE_DIR}/../DataProcessing
  {OpenCV_INCLUDE_DIRS}
)

# Link the firmware libraries built by parent
target_link_libraries(test_sift_time PRIVATE ${OpenCV_LIBS} dataProcessing)
target_link_libraries(test_fast_time PRIVATE ${OpenCV_LIBS} dataProcessing)

# Optionally set C++ standard
target_compile_features(test_sift_time PRIVATE cxx_std_11)
target_compile_features(test_fast_time PRIVATE cxx_std_11)
