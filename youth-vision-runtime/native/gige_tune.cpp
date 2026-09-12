// 读/调 GigE 相机的传输流控节点，定位取流带宽瓶颈
// 用法: gige_tune [serial] [set_packet_size] [set_throughput_limit]
#include <MvCameraControl.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

static std::string chars_to_string(const unsigned char* s, size_t n) {
  std::string out;
  for (size_t i = 0; i < n && s[i]; ++i) out.push_back(static_cast<char>(s[i]));
  return out;
}

static bool get_int(void* h, const char* node, long long* out) {
  MVCC_INTVALUE_EX v{};
  if (MV_CC_GetIntValueEx(h, node, &v) != MV_OK) return false;
  *out = static_cast<long long>(v.nCurValue);
  return true;
}

static void show_int(void* h, const char* node) {
  MVCC_INTVALUE_EX v{};
  if (MV_CC_GetIntValueEx(h, node, &v) == MV_OK)
    std::printf("  %-32s = %lld  (范围 %lld ~ %lld)\n", node,
                static_cast<long long>(v.nCurValue),
                static_cast<long long>(v.nMin),
                static_cast<long long>(v.nMax));
  else
    std::printf("  %-32s = <不支持>\n", node);
}

static void show_enum(void* h, const char* node) {
  MVCC_ENUMVALUE v{};
  if (MV_CC_GetEnumValue(h, node, &v) != MV_OK) {
    std::printf("  %-32s = <不支持>\n", node);
    return;
  }
  MVCC_ENUMENTRY e{};
  e.nValue = v.nCurValue;
  if (MV_CC_GetEnumEntrySymbolic(h, node, &e) == MV_OK)
    std::printf("  %-32s = %s (0x%x)\n", node, e.chSymbolic, v.nCurValue);
  else
    std::printf("  %-32s = 0x%x\n", node, v.nCurValue);
}

static void show_float(void* h, const char* node) {
  MVCC_FLOATVALUE v{};
  if (MV_CC_GetFloatValue(h, node, &v) == MV_OK)
    std::printf("  %-32s = %.2f\n", node, v.fCurValue);
  else
    std::printf("  %-32s = <不支持>\n", node);
}

static void dump(void* h) {
  show_int(h, "Width");
  show_int(h, "Height");
  show_int(h, "PayloadSize");
  show_int(h, "GevSCPSPacketSize");
  show_int(h, "GevSCPD");
  show_int(h, "GevSCBWR");
  show_int(h, "GevSCBWRA");
  show_int(h, "DeviceLinkThroughputLimit");
  show_enum(h, "DeviceLinkThroughputLimitMode");
  show_float(h, "AcquisitionFrameRate");
  show_float(h, "ResultingFrameRate");
}

int main(int argc, char** argv) {
  const std::string want_sn = argc > 1 ? argv[1] : "";
  const long long set_pkt = argc > 2 ? std::atoll(argv[2]) : 0;
  const long long set_thr = argc > 3 ? std::atoll(argv[3]) : 0;

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

  const int opt = MV_CC_GetOptimalPacketSize(h);
  std::printf("SDK 建议的最优包长 = %d\n", opt);

  std::printf("--- 调整前 ---\n");
  dump(h);

  bool changed = false;
  if (set_pkt > 0) {
    int r = MV_CC_SetIntValue(h, "GevSCPSPacketSize", static_cast<unsigned int>(set_pkt));
    std::printf("设置 GevSCPSPacketSize=%lld -> ret=0x%x\n", set_pkt, r);
    changed = true;
  } else if (opt > 0) {
    long long cur = 0;
    if (get_int(h, "GevSCPSPacketSize", &cur) && cur != opt) {
      int r = MV_CC_SetIntValue(h, "GevSCPSPacketSize", static_cast<unsigned int>(opt));
      std::printf("自动套用最优包长 %d -> ret=0x%x\n", opt, r);
      changed = true;
    }
  }
  if (set_thr > 0) {
    int r1 = MV_CC_SetEnumValueByString(h, "DeviceLinkThroughputLimitMode", "On");
    int r2 = MV_CC_SetIntValue(h, "DeviceLinkThroughputLimit", static_cast<unsigned int>(set_thr));
    std::printf("设置 ThroughputLimit=%lld -> mode_ret=0x%x limit_ret=0x%x\n", set_thr, r1, r2);
    changed = true;
  }

  if (changed) {
    std::printf("--- 调整后 ---\n");
    dump(h);
  }

  MV_CC_CloseDevice(h);
  MV_CC_DestroyHandle(h);
  MV_CC_Finalize();
  return 0;
}
