// 按序列号定位 GigE 相机并强制设置静态 IP（可逆：重跑本工具即可改回）
// 用法: gige_set_ip <serial> <ip> <mask> [gateway]
#include <MvCameraControl.h>

#include <cstdio>
#include <iostream>
#include <string>

static std::string chars_to_string(const unsigned char* s, size_t n) {
  std::string out;
  for (size_t i = 0; i < n && s[i]; ++i) out.push_back(static_cast<char>(s[i]));
  return out;
}

static unsigned int parse_ip(const char* s) {
  unsigned a = 0, b = 0, c = 0, d = 0;
  if (std::sscanf(s, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return 0;
  return (a << 24) | (b << 16) | (c << 8) | d;
}

static void print_ip(const char* label, unsigned int ip) {
  std::printf("%s=%u.%u.%u.%u", label, (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
              (ip >> 8) & 0xFF, ip & 0xFF);
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: gige_set_ip <serial> <ip> <mask> [gateway]\n";
    return 2;
  }
  const std::string want_sn = argv[1];
  const unsigned int ip = parse_ip(argv[2]);
  const unsigned int mask = parse_ip(argv[3]);
  const unsigned int gw = argc > 4 ? parse_ip(argv[4]) : 0;
  if (!ip || !mask) { std::cerr << "bad ip/mask\n"; return 2; }

  if (MV_CC_Initialize() != MV_OK) { std::cerr << "Init failed\n"; return 1; }
  MV_CC_DEVICE_INFO_LIST list{};
  if (MV_CC_EnumDevices(MV_GIGE_DEVICE, &list) != MV_OK) {
    std::cerr << "Enum failed\n";
    MV_CC_Finalize();
    return 1;
  }
  int idx = -1;
  for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
    auto* info = list.pDeviceInfo[i];
    if (info->nTLayerType != MV_GIGE_DEVICE) continue;
    if (chars_to_string(info->SpecialInfo.stGigEInfo.chSerialNumber,
                        sizeof(info->SpecialInfo.stGigEInfo.chSerialNumber)) == want_sn) {
      idx = static_cast<int>(i);
      break;
    }
  }
  if (idx < 0) {
    std::cerr << "serial " << want_sn << " not found (devices=" << list.nDeviceNum << ")\n";
    MV_CC_Finalize();
    return 1;
  }

  const auto& g = list.pDeviceInfo[idx]->SpecialInfo.stGigEInfo;
  std::printf("before: ");
  print_ip("ip", g.nCurrentIp);
  std::printf(" ");
  print_ip("mask", g.nCurrentSubNetMask);
  std::printf(" cfg=0x%x\n", g.nIpCfgCurrent);

  MV_CC_DEVICE_INFO* dev = list.pDeviceInfo[idx];
  const bool accessible = MV_CC_IsDeviceAccessible(dev, MV_ACCESS_Exclusive) != 0;
  std::printf("device accessible (same subnet)=%s\n", accessible ? "yes" : "no");

  void* handle = nullptr;
  int ret = MV_CC_CreateHandle(&handle, dev);
  if (ret != MV_OK) {
    std::cerr << "CreateHandle failed 0x" << std::hex << ret << "\n";
    MV_CC_Finalize();
    return 1;
  }

  // 同网段时先切静态模式；不同网段时 SDK 只能直接强推（官方 ForceIPEx 示例顺序）
  if (accessible) {
    ret = MV_GIGE_SetIpConfig(handle, MV_IP_CFG_STATIC);
    std::printf("SetIpConfig(STATIC) ret=0x%x %s\n", ret, ret == MV_OK ? "OK" : "FAIL");
  }

  ret = MV_GIGE_ForceIpEx(handle, ip, mask, gw);
  std::printf("ForceIpEx -> %s ret=0x%x %s\n", argv[2], ret, ret == MV_OK ? "OK" : "FAIL");

  if (accessible && ret == MV_OK) {
    // 官方示例：重建句柄后再设一次静态，使新 IP 落盘保存
    MV_CC_DestroyHandle(handle);
    handle = nullptr;
    dev->SpecialInfo.stGigEInfo.nCurrentIp = ip;
    dev->SpecialInfo.stGigEInfo.nCurrentSubNetMask = mask;
    dev->SpecialInfo.stGigEInfo.nDefultGateWay = gw;
    if (MV_CC_CreateHandle(&handle, dev) == MV_OK) {
      int r2 = MV_GIGE_SetIpConfig(handle, MV_IP_CFG_STATIC);
      std::printf("SetIpConfig(STATIC) after re-create ret=0x%x %s\n", r2,
                  r2 == MV_OK ? "OK" : "FAIL");
      MV_CC_DestroyHandle(handle);
      handle = nullptr;
    } else {
      std::cerr << "re-create handle failed\n";
    }
  } else if (handle) {
    MV_CC_DestroyHandle(handle);
  }

  MV_CC_Finalize();
  return ret == MV_OK ? 0 : 1;
}
