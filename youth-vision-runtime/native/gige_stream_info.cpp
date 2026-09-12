// 打印 GigE 相机当前视频流参数（像素格式/实际帧率/数据量），用于判断帧率瓶颈
#include <MvCameraControl.h>

#include <cstdio>
#include <iostream>
#include <string>

static std::string chars_to_string(const unsigned char* s, size_t n) {
  std::string out;
  for (size_t i = 0; i < n && s[i]; ++i) out.push_back(static_cast<char>(s[i]));
  return out;
}

static void show_enum(void* h, const char* node) {
  MVCC_ENUMVALUE v{};
  if (MV_CC_GetEnumValue(h, node, &v) == MV_OK) {
    MVCC_ENUMENTRY e{};
    e.nValue = v.nCurValue;
    if (MV_CC_GetEnumEntrySymbolic(h, node, &e) == MV_OK)
      std::printf("  %s = %s (0x%x)\n", node, e.chSymbolic, v.nCurValue);
    else
      std::printf("  %s = 0x%x\n", node, v.nCurValue);
  } else {
    std::printf("  %s = <读取失败>\n", node);
  }
}

static void show_float(void* h, const char* node) {
  MVCC_FLOATVALUE v{};
  if (MV_CC_GetFloatValue(h, node, &v) == MV_OK)
    std::printf("  %s = %.3f  (范围 %.3f ~ %.3f)\n", node, v.fCurValue, v.fMin, v.fMax);
  else
    std::printf("  %s = <不支持/读取失败>\n", node);
}

static void show_bool(void* h, const char* node) {
  bool b = false;
  if (MV_CC_GetBoolValue(h, node, &b) == MV_OK)
    std::printf("  %s = %s\n", node, b ? "true" : "false");
  else
    std::printf("  %s = <不支持>\n", node);
}

static void show_int(void* h, const char* node) {
  MVCC_INTVALUE_EX v{};
  if (MV_CC_GetIntValueEx(h, node, &v) == MV_OK)
    std::printf("  %s = %lld\n", node, static_cast<long long>(v.nCurValue));
  else
    std::printf("  %s = <不支持>\n", node);
}

int main(int argc, char** argv) {
  const std::string want_sn = argc > 1 ? argv[1] : "";
  if (MV_CC_Initialize() != MV_OK) { std::cerr << "Init failed\n"; return 1; }
  MV_CC_DEVICE_INFO_LIST list{};
  if (MV_CC_EnumDevices(MV_GIGE_DEVICE, &list) != MV_OK) {
    std::cerr << "Enum failed\n"; MV_CC_Finalize(); return 1;
  }
  int idx = -1;
  for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
    auto* info = list.pDeviceInfo[i];
    if (info->nTLayerType != MV_GIGE_DEVICE) continue;
    if (want_sn.empty() ||
        chars_to_string(info->SpecialInfo.stGigEInfo.chSerialNumber,
                        sizeof(info->SpecialInfo.stGigEInfo.chSerialNumber)) == want_sn) {
      idx = static_cast<int>(i);
      break;
    }
  }
  if (idx < 0) { std::cerr << "device not found\n"; MV_CC_Finalize(); return 1; }

  void* h = nullptr;
  if (MV_CC_CreateHandle(&h, list.pDeviceInfo[idx]) != MV_OK) { MV_CC_Finalize(); return 1; }
  if (MV_CC_OpenDevice(h, MV_ACCESS_Exclusive, 0) != MV_OK) {
    std::cerr << "OpenDevice failed\n"; MV_CC_DestroyHandle(h); MV_CC_Finalize(); return 1;
  }

  std::printf("当前流参数:\n");
  show_int(h, "Width");
  show_int(h, "Height");
  show_enum(h, "PixelFormat");
  show_int(h, "PayloadSize");
  show_bool(h, "AcquisitionFrameRateEnable");
  show_float(h, "AcquisitionFrameRate");
  show_float(h, "ResultingFrameRate");
  show_enum(h, "ExposureAuto");
  show_float(h, "ExposureTime");
  show_float(h, "GevSCPSPacketSize");
  show_int(h, "GevSCPD");

  MV_CC_CloseDevice(h);
  MV_CC_DestroyHandle(h);
  MV_CC_Finalize();
  return 0;
}
