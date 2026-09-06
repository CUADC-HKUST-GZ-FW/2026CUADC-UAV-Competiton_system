#include <MvCameraControl.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

static void check(int ret, const char* what) {
  if (ret != MV_OK) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s failed: 0x%x", what, ret);
    throw std::runtime_error(buf);
  }
}

static std::string chars_to_string(const unsigned char* s, size_t n) {
  std::string out;
  for (size_t i = 0; i < n && s[i]; ++i) out.push_back(static_cast<char>(s[i]));
  return out;
}

int main(int argc, char** argv) {
  int frames = argc > 1 ? std::atoi(argv[1]) : 120;
  float fps = argc > 2 ? std::atof(argv[2]) : 60.0f;
  float exposure_us = argc > 3 ? std::atof(argv[3]) : 2500.0f;

  void* handle = nullptr;
  try {
    check(MV_CC_Initialize(), "MV_CC_Initialize");
    MV_CC_DEVICE_INFO_LIST list{};
    check(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list), "MV_CC_EnumDevices");
    std::cout << "devices=" << list.nDeviceNum << "\n";
    if (list.nDeviceNum == 0) return 3;
    auto* info = list.pDeviceInfo[0];
    if (info->nTLayerType == MV_USB_DEVICE) {
      std::cout << "type=USB3Vision model="
                << chars_to_string(info->SpecialInfo.stUsb3VInfo.chModelName, sizeof(info->SpecialInfo.stUsb3VInfo.chModelName))
                << " serial="
                << chars_to_string(info->SpecialInfo.stUsb3VInfo.chSerialNumber, sizeof(info->SpecialInfo.stUsb3VInfo.chSerialNumber))
                << "\n";
    }
    check(MV_CC_CreateHandle(&handle, info), "MV_CC_CreateHandle");
    check(MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0), "MV_CC_OpenDevice");
    MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
    MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
    MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", fps);
    MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
    MV_CC_SetFloatValue(handle, "ExposureTime", exposure_us);
    MV_CC_SetImageNodeNum(handle, 4);
    check(MV_CC_StartGrabbing(handle), "MV_CC_StartGrabbing");

    int ok = 0;
    int width = 0;
    int height = 0;
    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < frames; ++i) {
      MV_FRAME_OUT frame{};
      int ret = MV_CC_GetImageBuffer(handle, &frame, 1000);
      if (ret != MV_OK) {
        std::cerr << "GetImageBuffer failed: 0x" << std::hex << ret << std::dec << "\n";
        continue;
      }
      width = static_cast<int>(frame.stFrameInfo.nWidth);
      height = static_cast<int>(frame.stFrameInfo.nHeight);
      std::vector<unsigned char> bgr(static_cast<size_t>(width) * height * 3);
      MV_CC_PIXEL_CONVERT_PARAM conv{};
      conv.nWidth = frame.stFrameInfo.nWidth;
      conv.nHeight = frame.stFrameInfo.nHeight;
      conv.pSrcData = frame.pBufAddr;
      conv.nSrcDataLen = frame.stFrameInfo.nFrameLen;
      conv.enSrcPixelType = frame.stFrameInfo.enPixelType;
      conv.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
      conv.pDstBuffer = bgr.data();
      conv.nDstBufferSize = static_cast<unsigned int>(bgr.size());
      ret = MV_CC_ConvertPixelType(handle, &conv);
      MV_CC_FreeImageBuffer(handle, &frame);
      if (ret != MV_OK) {
        std::cerr << "ConvertPixelType failed: 0x" << std::hex << ret << std::dec << "\n";
        continue;
      }
      ++ok;
      if (ok == 1) {
        cv::Mat image(height, width, CV_8UC3, bgr.data());
        cv::imwrite("/tmp/mvs_probe_first.jpg", image);
      }
    }
    auto stop = std::chrono::steady_clock::now();
    double sec = std::chrono::duration<double>(stop - start).count();
    std::cout << "frames_ok=" << ok << " wall_sec=" << sec << " fps=" << (ok / sec)
              << " size=" << width << "x" << height << " first=/tmp/mvs_probe_first.jpg\n";

    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "fatal: " << e.what() << "\n";
    if (handle) {
      MV_CC_StopGrabbing(handle);
      MV_CC_CloseDevice(handle);
      MV_CC_DestroyHandle(handle);
    }
    MV_CC_Finalize();
    return 1;
  }
}
