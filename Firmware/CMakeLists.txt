cmake_minimum_required(VERSION 3.15)
project(ViOdFeatExt_Firmware LANGUAGES C CXX)

# Add component directories
add_subdirectory(I2C)
add_subdirectory(ICM20948)

message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Build tests: ${BUILD_TESTS}")
