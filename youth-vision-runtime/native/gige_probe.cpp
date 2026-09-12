// GigE 相机网络参数探针：打印每台 MVS 设备的型号/序列号/当前 IP/掩码/网关
#include <MvCameraControl.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

static std::string chars_to_string(const unsigned char* s, size_t n) {
  std::string out;
  for (size_t i = 0; i < n && s[i]; ++i) out.push_back(static_cast<char>(s[i]));
  return out;
}

static void print_ip(const char* label, unsigned int ip) {
  std::printf("  %s=%u.%u.%u.%u\n", label, (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
              (ip >> 8) & 0xFF, ip & 0xFF);
}

int main(int argc, char** argv) {
  const std::string want_sn = argc > 1 ? argv[1] : "";
  if (MV_CC_Initialize() != MV_OK) { std::cerr << "Init failed\n"; return 1; }
  MV_CC_DEVICE_INFO_LIST list{};
  if (MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list) != MV_OK) {
    std::cerr << "Enum failed\n"; return 1;
  }
  std::printf("devices=%u\n", list.nDeviceNum);
  int want_idx = -1;
  for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
    auto* info = list.pDeviceInfo[i];
    if (info->nTLayerType == MV_GIGE_DEVICE) {
      const auto& g = info->SpecialInfo.stGigEInfo;
      std::string sn = chars_to_string(g.chSerialNumber, sizeof(g.chSerialNumber));
      std::printf("[%u] GIGE model=%s serial=%s\n", i,
                  chars_to_string(g.chModelName, sizeof(g.chModelName)).c_str(), sn.c_str());
      print_ip("current_ip", g.nCurrentIp);
      print_ip("current_mask", g.nCurrentSubNetMask);
      print_ip("gateway", g.nDefultGateWay);
      print_ip("net_export", g.nNetExport);
      print_ip("host_ip", g.nHostIP);
      print_ip("multicast_ip", g.nMulticastIP);
      std::printf("  ip_cfg_option=0x%x ip_cfg_current=0x%x gentl_type=%u mcast_port=%u\n",
                  g.nIpCfgOption, g.nIpCfgCurrent, g.nGenTLType, g.nMulticastPort);
      if (want_sn.empty() || sn == want_sn) want_idx = static_cast<int>(i);
    } else if (info->nTLayerType == MV_USB_DEVICE) {
      const auto& u = info->SpecialInfo.stUsb3VInfo;
      std::printf("[%u] USB model=%s serial=%s\n", i,
                  chars_to_string(u.chModelName, sizeof(u.chModelName)).c_str(),
                  chars_to_string(u.chSerialNumber, sizeof(u.chSerialNumber)).c_str());
    } else {
      std::printf("[%u] OTHER tlayer=0x%x\n", i, info->nTLayerType);
    }
  }
  if (want_idx >= 0) {
    void* handle = nullptr;
    int ret = MV_CC_CreateHandle(&handle, list.pDeviceInfo[want_idx]);
    std::printf("CreateHandle[%d] ret=0x%x\n", want_idx, ret);
    if (ret == MV_OK) {
      ret = MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0);
      std::printf("OpenDevice ret=0x%x (%s)\n", ret, ret == MV_OK ? "OK" : "FAIL");
      if (ret == MV_OK) MV_CC_CloseDevice(handle);
      MV_CC_DestroyHandle(handle);
    }
  }
  MV_CC_Finalize();
  return 0;
}
