cmake_minimum_required(VERSION 3.10)
project(firmware_tests_examples LANGUAGES C CXX)

find_package(Threads REQUIRED)
# trying to get opecv to work
find_package(OpenCV REQUIRED)


# Build only test_imu explicitly:
add_executable(test_imu ${CMAKE_CURRENT_SOURCE_DIR}/imu_test.cpp)
add_executable(test_capture_device ${CMAKE_CURRENT_SOURCE_DIR}/cam_test.cpp)
add_executable(test_full_capture ${CMAKE_CURRENT_SOURCE_DIR}/capture_test.cpp)
add_executable(test_webcamMT ${CMAKE_CURRENT_SOURCE_DIR}/webcamMultiThreadedTest.cpp)
add_executable(test_stereocamMT ${CMAKE_CURRENT_SOURCE_DIR}/stereoCamMultiThreadedTest.cpp)
add_executable(test_sift_time ${CMAKE_CURRENT_SOURCE_DIR}/sift_time_test.cpp)
add_executable(test_fast_time ${CMAKE_CURRENT_SOURCE_DIR}/fast_test.cpp)


# Ensure headers are findable
target_include_directories(test_imu PRIVATE
  ${CMAKE_SOURCE_DIR}/../I2C
  ${CMAKE_SOURCE_DIR}/../ICM20948
)

# Ensure headers are findable
target_include_directories(test_capture_device PRIVATE
  ${CMAKE_SOURCE_DIR}/../Capture
)

# Ensure headers are findable
target_include_directories(test_full_capture PRIVATE
  ${CMAKE_SOURCE_DIR}/../I2C
  ${CMAKE_SOURCE_DIR}/../ICM20948
  ${CMAKE_SOURCE_DIR}/../Capture
)

# Ensure headers are findable
target_include_directories(test_webcamMT PRIVATE
  ${CMAKE_SOURCE_DIR}/../Capture
)

# Ensure headers are findable
target_include_directories(test_stereocamMT PRIVATE
  ${CMAKE_SOURCE_DIR}/../I2C
  ${CMAKE_SOURCE_DIR}/../ICM20948
  ${CMAKE_SOURCE_DIR}/../Capture
)

# fixing opencv to work for me
target_link_libraries(test_sift_time ${OpenCV_LIBS})
target_include_directories(test_sift_time PRIVATE 
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(test_fast_time ${OpenCV_LIBS})
target_include_directories(test_fast_time PRIVATE 
  ${OpenCV_INCLUDE_DIRS}
)


# Link the firmware libraries built by parent (i2c and icm20948)
target_link_libraries(test_imu PRIVATE i2c icm20948)
target_link_libraries(test_capture_device PRIVATE captureDevice)
target_link_libraries(test_full_capture PRIVATE i2c icm20948 captureDevice)
target_link_libraries(test_webcamMT PRIVATE captureDevice Threads::Threads)
target_link_libraries(test_stereocamMT PRIVATE i2c icm20948 captureDevice Threads::Threads)

# Optionally set C++ standard
target_compile_features(test_imu PRIVATE cxx_std_11)
target_compile_features(test_capture_device PRIVATE cxx_std_11)
target_compile_features(test_full_capture PRIVATE cxx_std_11)
target_compile_features(test_webcamMT PRIVATE cxx_std_11)
target_compile_features(test_stereocamMT PRIVATE cxx_std_11)
