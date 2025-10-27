cmake_minimum_required(VERSION 3.15)

project(icm20948_component LANGUAGES C CXX)

# If there are implementation files in this directory, create a static lib.
file(GLOB ICM_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS "*.c" "*.cpp" "*.cc" "*.cxx")

if(ICM_SOURCES)
  add_library(icm20948 STATIC ${ICM_SOURCES})
  # Expose this directory for includes in case consumers need it
  target_include_directories(icm20948 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  # Ensure a sane C standard for consumers
  target_compile_features(icm20948 PUBLIC c_std_11)

  # Link against the i2c component so icm20948 gets i2c's include dirs and symbols
  # i2c must be added by the top-level CMakeLists (add_subdirectory(I2C) before this)
  target_link_libraries(icm20948 PUBLIC i2c)
else()
  # No sources yet: make an INTERFACE target so other targets can link/include.
  add_library(icm20948 INTERFACE)
  target_include_directories(icm20948 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

  # If someone links to icm20948, also pull in i2c's include dirs (keeps linkage consistent)
  target_link_libraries(icm20948 INTERFACE i2c)
endif()
