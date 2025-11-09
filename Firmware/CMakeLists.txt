cmake_minimum_required(VERSION 3.10)
project(ViOdFeatExt_Firmware LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
# Add component directories
add_subdirectory(I2C)
add_subdirectory(ICM20948)
add_subdirectory(Capture)

# Build executables in tests/ (one per .cpp). Enable by uncommenting the line below.
# If you prefer to keep tests/ ignored, leave this commented.
add_subdirectory(tests)

message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Build tests: ${BUILD_TESTS}")


