#include "CaptureDevice.h"
#include "DataStorageHandler.h"
#include "icm20948.h"

#include <stdio.h>
#include <time.h>
#include <unistd.h>

/**
 * Prints help information about this application.
 *  @param name the name of the executable.
 */
void printHelp(const char *name) {
  printf("Usage: %s [options] \n", name);
  printf("    -h, --help  -> prints this information \n");
  printf("    -l, --loop  -> sets the maximum number of loop iterations \n");
  printf("    -c, --calib -> if set to one triggers gyro calibration \n");
  printf("    -f, --freq  -> set frequency of capture in Hz\n");
  printf("\nExample: %s -l 2000 -c 1 \n", name);
}
/**
 *  @return the time of the system in seconds.
 */
double getTime() {
  struct timespec timeStruct;
  clock_gettime(CLOCK_REALTIME, &timeStruct);
  return static_cast<double>(timeStruct.tv_sec) +
         static_cast<double>(timeStruct.tv_nsec / 1000000) / 1000;
}

int main(int argc, char **argv) {
  /** Current time in seconds to calculate time needed to sleep for the main
   * loop. */
  double currentTime;
  /** The maximum number of loops. */
  ulong max = 200;
  /**Loop period*/
  double period = 0.2;

  /** Time to sleep in milliseconds for the main loop. */
  int sleepTime;
  /** IMU implementation. */
  ICM20948 imu;
  /** Pointer for IMU data.  */
  const IMUData *data;
  /** Flag to indicate of gyro calibration should be performed. */
  bool calib = false;

  /**Stereo camera device implementation*/
  CaptureDevice camDev;
  /**Stereo camera data*/
  cv::Mat left, right;

  /** Data storage handler*/
  DataStorageHandler dH;

  /* Parse any inputs that may have been provided. */
  for (int i = 1; i < static_cast<ulong>(argc); ++i) {
    if ((0 == strcmp(argv[i], "--loop")) || (0 == strcmp(argv[i], "-l"))) {
      max = static_cast<ulong>(atol(argv[i + 1]));
    } else if ((0 == strcmp(argv[i], "--calib")) ||
               (0 == strcmp(argv[i], "-c"))) {
      calib = static_cast<bool>(atoi(argv[i + 1]));
    } else if ((0 == strcmp(argv[i], "--freq")) ||
               (0 == strcmp(argv[i], "-f"))) {
      period = 1 / static_cast<double>(atoi(argv[i + 1]));
    } else if ((0 == strcmp(argv[i], "--help")) ||
               (0 == strcmp(argv[i], "-h"))) {
      printHelp(argv[0]);
      return 0;
    }
  }

  // initialisation error handling
  try {
    dH.Init();
    camDev.Init();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  if (!imu.initialise()) {
    puts("Failed to detect ICM-20948");
    return 1;
  }

  // Calibrating if configured
  if (calib) {
    puts("Performing gyroscope calibration");
    imu.calibrateGyro();
  }

  for (int i = 0; i < max; ++i) {
    currentTime = getTime();

    /**Get data*/
    try {
      data = &imu.imuDataGet();
      left = camDev.GetLeftImage();
      right = camDev.GetRightImage();
    } catch (const std::exception &e) {
      std::cerr << e.what() << '\n';
      return 1;
    }

    std::string leftName = dH.SaveImage(left, "left");
    std::string rightName = dH.SaveImage(right, "right");

    dH.SaveData(0, 0, data->mAcc[0], data->mAcc[1], data->mAcc[2],
                data->mGyro[0], data->mGyro[1], data->mGyro[2], data->mMag[0],
                data->mMag[1], data->mMag[2], leftName, rightName);

    /* Calculate sleep time by taking expected period between IMU updates and
     * reducing it by a time needed to process data. */
    sleepTime =
        static_cast<int>(((period) - (getTime() - currentTime)) * 1000000.0);
    /* We use signed type in case processing takes more than expected period
     * between updates. */
    if (sleepTime > 0)
      usleep(static_cast<uint>(sleepTime));
    else
      std::cerr << "WARNING: Unable to keep configured period by "
                << std::to_string(sleepTime / 1000000.0) << '\n';
  }

  return 0;
}
