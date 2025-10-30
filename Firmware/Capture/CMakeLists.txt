cmake_minimum_required(VERSION 3.15)

project(CaptureDevice LANGUAGES C CXX)

find_package(OpenCV REQUIRED)
message("OpenCV Include Path: ${OpenCV_INCLUDE_DIRS}")

# Sources
set(CAPTUREDEVICE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/CaptureDevice.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStorageHandler.cpp
)

add_library(captureDevice STATIC ${CAPTUREDEVICE_SOURCES})

target_include_directories(captureDevice
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(captureDevice ${OpenCV_LIBS})

# Ensure a sane C standard for consumers
target_compile_features(i2c PUBLIC c_std_17)
