cmake_minimum_required(VERSION 3.15)
project(firmware_tests_examples LANGUAGES C CXX)

# Build only test_imu explicitly:
add_executable(test_imu ${CMAKE_CURRENT_SOURCE_DIR}/test_imu.cpp)

# Ensure headers are findable
target_include_directories(test_imu PRIVATE
  ${CMAKE_SOURCE_DIR}/../I2C
  ${CMAKE_SOURCE_DIR}/../ICM20948
)

# Link the firmware libraries built by parent (i2c and icm20948)
target_link_libraries(test_imu PRIVATE i2c icm20948)

# Optionally set C++ standard
target_compile_features(test_imu PRIVATE cxx_std_11)
