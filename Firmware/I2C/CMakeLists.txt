cmake_minimum_required(VERSION 3.15)

project(i2c_component LANGUAGES C CXX)

# Sources
set(I2C_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/i2c.cpp
)

add_library(i2c STATIC ${I2C_SOURCES})

target_include_directories(i2c
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Ensure a sane C standard for consumers
target_compile_features(i2c PUBLIC c_std_11)
