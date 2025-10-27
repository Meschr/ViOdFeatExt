# How to build

This folder contains the firmware component sources and the CMake build files for building those components on the host (Linux/macOS/Windows) and for producing host-side test executables (for example `test_imu`).

This README explains how to build the firmware libraries and how to compile/run a test executable such as `test_imu`.

Prerequisites

- CMake >= 3.15
- A C/C++ compiler (g++, clang++, MSVC)
- (Optional) arm-none-eabi toolchain and a linker script if you later want to cross-compile for an MCU

Repository layout (relevant)

- I2C/ — i2c.cpp, i2c.h and CMakeLists for the i2c static library
- ICM20948/ — icm20948 sources and CMakeLists (creates a target or INTERFACE target)
- tests/ — host test mains (e.g. test_imu.cpp) and a tests/CMakeLists to build them
- CMakeLists.txt — top-level CMake for Firmware that wires subdirs together
- .gitignore — excludes build artifacts

Recommended build (out-of-source)

1. From the repository root (or from Firmware/), create a build directory:
   mkdir -p Firmware/build
   cd Firmware/build

2. Configure with CMake:
   cmake ..

   - This configures the I2C and ICM20948 targets. If the tests subdirectory has a CMakeLists and you have `add_subdirectory(tests)` in Firmware/CMakeLists.txt, executables for each .cpp in tests/ will also be configured.

3. Build everything (or a specific target):
   cmake --build .

   Build a single target (e.g. only the i2c library):
   cmake --build . --target i2c

   Build only the test_imu executable:
   cmake --build . --target test_imu

Where the binaries appear

- Exact locations depend on your CMake generator. With the default Unix Makefiles the test executables are normally placed in the build root (e.g. Firmware/build/) or under build/tests/ if the CMakeLists sets a subdirectory output.
- To quickly find a built executable:
  find . -type f -executable -name test_imu

Running test_imu:

- From the build directory:
  ./test_imu
  or, if placed under tests/:
  ./tests/test_imu

Clean build:

- rm -rf Firmware/build
- mkdir -p Firmware/build && cd Firmware/build
- cmake .. && cmake --build .

# Install and verify i2c-tools

Firstly, install i2c-tools. Run these commands in a terminal:

```bash
sudo apt-get update
sudo apt-get install -y i2c-tools
```

Check the installation status with:

```bash
apt-cache policy i2c-tools
```

A successful installation will produce output similar to:

```
i2c-tools:
  was installed: 4.0-2
  candidate:    4.0-2
Version List:
*** 4.0-2 500
500 http://ports.ubuntu.com/ubuntu-ports bionic/universe arm64 Packages
100 /var/lib/dpkg/status
```

Scan all I²C devices on a given bus and print their addresses. For example, to scan bus 1:

```bash
sudo i2cdetect -y -r -a 1
```

The ICM20948 should show up with adress: `0x68`

Notes and tips:

- Replace `1` with the correct bus number for your hardware (common values are `0` or `1`).
- `i2cdetect` requires root privileges (hence `sudo`).
- On some systems (e.g., Raspberry Pi) you may need to enable I2C via system configuration (raspi-config) or load kernel modules such as `i2c-dev`.
- Use `man i2cdetect` or `i2c-tools` documentation for more options and details.
