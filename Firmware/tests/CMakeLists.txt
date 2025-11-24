cmake_minimum_required(VERSION 3.10)
project(firmware_tests_examples LANGUAGES C CXX)

find_package(Threads REQUIRED)

# Build only test_imu explicitly:
add_executable(test_imu ${CMAKE_CURRENT_SOURCE_DIR}/imu_test.cpp)
add_executable(test_capture_device ${CMAKE_CURRENT_SOURCE_DIR}/cam_test.cpp)
add_executable(test_full_capture ${CMAKE_CURRENT_SOURCE_DIR}/capture_test.cpp)
add_executable(test_webcamMT ${CMAKE_CURRENT_SOURCE_DIR}/webcamMultiThreadedTest.cpp)
add_executable(test_stereocamMT ${CMAKE_CURRENT_SOURCE_DIR}/stereoCamMultiThreadedTest.cpp)
add_executable(checkboard_calibration ${CMAKE_CURRENT_SOURCE_DIR}/checkboard_calibration.cpp)
add_executable(approx_calibration ${CMAKE_CURRENT_SOURCE_DIR}/approximationCalibration.cpp)


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

# Ensure headers are findable
target_include_directories(checkboard_calibration PRIVATE
${CMAKE_SOURCE_DIR}/../DataProcessing
)

# Ensure headers are findable
target_include_directories(approx_calibration PRIVATE
${CMAKE_SOURCE_DIR}/../DataProcessing
)

# Link the firmware libraries built by parent (i2c and icm20948)
target_link_libraries(test_imu PRIVATE i2c icm20948)
target_link_libraries(test_capture_device PRIVATE captureDevice)
target_link_libraries(test_full_capture PRIVATE i2c icm20948 captureDevice)
target_link_libraries(test_webcamMT PRIVATE captureDevice Threads::Threads)
target_link_libraries(test_stereocamMT PRIVATE i2c icm20948 captureDevice Threads::Threads)
target_link_libraries(checkboard_calibration PRIVATE dataProcessing)
target_link_libraries(approx_calibration PRIVATE dataProcessing)

# Optionally set C++ standard
target_compile_features(test_imu PRIVATE cxx_std_11)
target_compile_features(test_capture_device PRIVATE cxx_std_11)
target_compile_features(test_full_capture PRIVATE cxx_std_11)
target_compile_features(test_webcamMT PRIVATE cxx_std_11)
target_compile_features(test_stereocamMT PRIVATE cxx_std_11)
target_compile_features(checkboard_calibration PRIVATE cxx_std_11)
target_compile_features(approx_calibration PRIVATE cxx_std_11)
