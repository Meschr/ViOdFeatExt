cmake_minimum_required(VERSION 3.15)
project(ViOdFeatExt_Firmware LANGUAGES C CXX)

# Add component directories
add_subdirectory(I2C)
add_subdirectory(ICM20948)

# Build executables in tests/ (one per .cpp). Enable by uncommenting the line below.
# If you prefer to keep tests/ ignored, leave this commented.
add_subdirectory(tests)

message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Build tests: ${BUILD_TESTS}")
