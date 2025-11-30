#include <CaptureDevice.h>
#include <DataStorageHandler.h>
#include <stdio.h>

int main(int argc, char **argv) {
  printf("Testing Camera!\n");

  DataStorageHandler dH;
  dH.Init();

  // int picCnt = 0;
  CaptureDevice camDev;
  camDev.Init();
  camDev.GetFpsRightCamera();
  camDev.GetFpsLeftCamera();

  for (int i = 0; i < 20; ++i) { // capture 10 frames
    cv::Mat left, right /*, stereo*/;

    left = camDev.GetLeftImage();
    right = camDev.GetRightImage();
    // stereo = camDev.GetStereoImage();

    std::string leftName = dH.SaveImage(left, "left");
    std::string rightName = dH.SaveImage(right, "right");
    // std::string stereoName = dH.SaveImage(stereo, "stereo");

    dH.SaveData(0,0,1.9, 2.8, 3.7, 4.6, 5.5, 6.4, 7.2, 8.1, 9.0, 
                leftName,
                rightName/*,
                stereoName*/);

    std::cout << "Saved frame " << i << std::endl;
  }

  std::cout << "Capture complete.\n";
  return 0;
}
