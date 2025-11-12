#include "icm20948.h"
#include <CaptureDevice.h>
#include <DataStorageHandler.h>
#include <chrono>
#include <future>
#include <iostream>
#include <optional>
#include <thread>

#include "ThreadSafeQueue.h"

// --- Configuration ---
uint maxRunDuration = 500;
double targetFPS = 30.0;
bool gyroCalib = false;

/**
 * Prints help information about this application.
 *  @param name the name of the executable.
 */
void printHelp(const char *name) {
  printf("Usage: %s [options] \n", name);
  printf("    -h, --help  -> prints this information \n");
  printf("    -l, --loop  -> sets the time of the recording in seconds \n");
  printf("    -c, --calib -> if set to one triggers gyro calibration \n");
  printf("    -f, --freq  -> set frequency of capture in Hz\n");
  printf("\nExample: %s -l 200 -c 1 -f 30\n", name);
}

ThreadSafeQueue<std::optional<DataPacket>> data_queue;

void data_capture(double target_fps, std::future<void> startSignal) {
  std::cout << "[Capture] Starting. Target FPS: " << target_fps
            << " | Capture duration: " << maxRunDuration << std::endl;

  startSignal.wait();

  CaptureDevice camDev;
  try {
    camDev.Init();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return;
  }

  // TODO: here fps check with mac fps of camera if viable. if not adjust or
  // abort

  ICM20948 imu;
  if (!imu.initialise()) {
    puts("Failed to detect ICM-20948");
    return;
  }

  // Calibrating if configured
  if (gyroCalib) {
    puts("Performing gyroscope calibration");
    imu.calibrateGyro();
  }

  auto frame_duration =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / target_fps));
  int frame_counter = 0;

  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::duration_cast<std::chrono::seconds>(
             std::chrono::steady_clock::now() - start_time)
             .count() < maxRunDuration) {
    auto loop_start_time = std::chrono::steady_clock::now();

    // 1. CAPTURE DATA
    DataPacket packet;
    packet.frameNumber = frame_counter;
    packet.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();

    auto imuData = &imu.imuDataGet();
    packet.accX = imuData->mAcc[0];
    packet.accY = imuData->mAcc[1];
    packet.accZ = imuData->mAcc[2];
    packet.gyroX = imuData->mGyro[0];
    packet.gyroY = imuData->mGyro[1];
    packet.gyroZ = imuData->mGyro[2];
    packet.magX = imuData->mMag[0];
    packet.magY = imuData->mMag[1];
    packet.magZ = imuData->mMag[2];
    packet.leftImage = camDev.GetLeftImage();
    packet.rightImage = camDev.GetRightImage();
    std::cout << "[Capture] Saved frame " << frame_counter << std::endl;

    // 2. PUSH TO QUEUE
    data_queue.push(packet);
    frame_counter++;

    // 3. MAINTAIN FPS
    auto elapsed = std::chrono::steady_clock::now() - loop_start_time;
    auto sleep_time = frame_duration - elapsed;
    if (sleep_time > std::chrono::steady_clock::duration::zero()) {
      std::this_thread::sleep_for(sleep_time);
    }
  }

  std::cout << "[Capture] Finished capturing. Sending stop signal."
            << std::endl;
  data_queue.push(std::nullopt); // Push a null optional to signal stop
}

void data_storage(std::promise<void> startSignal) {
  DataStorageHandler dH;
  dH.Init();

  startSignal.set_value();
  std::cout
      << "[Storage] LogData directory created. Unblocking capture thread!\n";

  while (true) {
    std::optional<DataPacket> packet_opt = data_queue.wait_and_pop();

    if (!packet_opt.has_value()) {
      std::cout << "[Storage] Stop signal received. Exiting." << std::endl;
      break;
    }

    DataPacket packet = packet_opt.value();
    std::string leftName = dH.SaveImage(packet.leftImage, "left");
    std::string rightName = dH.SaveImage(packet.rightImage, "right");

    dH.SaveData(packet.timestamp, packet.frameNumber, packet.accX, packet.accY,
                packet.accZ, packet.gyroX, packet.gyroY, packet.gyroZ,
                packet.magX, packet.magY, packet.magZ, leftName, rightName);
  }

  std::cout << "[Storage] Finished writing all data." << std::endl;
}

int main(int argc, char **argv) {
  for (int i = 1; i < static_cast<ulong>(argc); ++i) {
    if ((0 == strcmp(argv[i], "--loop")) || (0 == strcmp(argv[i], "-l"))) {
      maxRunDuration = static_cast<ulong>(atol(argv[i + 1]));
    } else if ((0 == strcmp(argv[i], "--calib")) ||
               (0 == strcmp(argv[i], "-c"))) {
      gyroCalib = static_cast<bool>(atoi(argv[i + 1]));
    } else if ((0 == strcmp(argv[i], "--freq")) ||
               (0 == strcmp(argv[i], "-f"))) {
      targetFPS = static_cast<double>(atoi(argv[i + 1]));
    } else if ((0 == strcmp(argv[i], "--help")) ||
               (0 == strcmp(argv[i], "-h"))) {
      printHelp(argv[0]);
      return 0;
    }
  }

  std::cout << "[Main] Starting Producer and Consumer threads..." << std::endl;

  std::promise<void> startPromise;
  std::future<void> startFuture = startPromise.get_future();

  std::thread capture_thread(data_capture, targetFPS, std::move(startFuture));
  std::thread storage_thread(data_storage, std::move(startPromise));

  storage_thread.join();
  std::cout << "[Main] Storage thread has joined." << std::endl;
  capture_thread.join();
  std::cout << "[Main] Capture thread has joined." << std::endl;

  std::cout << "[Main] All done!" << std::endl;
  return 0;
}
