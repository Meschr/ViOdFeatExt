cmake_minimum_required(VERSION 3.15)
project(firmware_tests_examples LANGUAGES C CXX)

# Build only test_imu explicitly:
add_executable(test_imu ${CMAKE_CURRENT_SOURCE_DIR}/imu_test.cpp)
add_executable(test_capture_device ${CMAKE_CURRENT_SOURCE_DIR}/cam_test.cpp)

# Ensure headers are findable
target_include_directories(test_imu PRIVATE
  ${CMAKE_SOURCE_DIR}/../I2C
  ${CMAKE_SOURCE_DIR}/../ICM20948
)

# Ensure headers are findable
target_include_directories(test_capture_device PRIVATE
  ${CMAKE_SOURCE_DIR}/../Capture
)

# Link the firmware libraries built by parent (i2c and icm20948)
target_link_libraries(test_imu PRIVATE i2c icm20948)
target_link_libraries(test_capture_device PRIVATE captureDevice)

# Optionally set C++ standard
target_compile_features(test_imu PRIVATE cxx_std_11)
target_compile_features(test_capture_device PRIVATE cxx_std_11)
