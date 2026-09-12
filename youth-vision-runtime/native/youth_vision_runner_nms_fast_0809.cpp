#include <NvInfer.h>
#include <MvCameraControl.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;

class Logger final : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= Severity::kWARNING) std::cerr << "[TRT] " << msg << "\n";
  }
};

static void check_cuda(cudaError_t e, const char* what) {
  if (e != cudaSuccess) {
    std::cerr << "CUDA error at " << what << ": " << cudaGetErrorString(e) << "\n";
    std::exit(2);
  }
}

static std::string trim(const std::string& s) {
  const char* ws = " \t\r\n\"'";
  size_t b = s.find_first_not_of(ws);
  if (b == std::string::npos) return "";
  size_t e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}

static std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

static bool starts_with(const std::string& s, const std::string& p) {
  return s.rfind(p, 0) == 0;
}

static std::vector<char> read_file_binary(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open file: " + path);
  f.seekg(0, std::ios::end);
  std::vector<char> data(static_cast<size_t>(f.tellg()));
  f.seekg(0, std::ios::beg);
  f.read(data.data(), static_cast<std::streamsize>(data.size()));
  return data;
}

static std::string read_text_file(const std::string& path) {
  std::ifstream f(path);
  if (!f) return "";
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

static std::vector<std::string> read_lines(const std::string& path) {
  std::ifstream f(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(f, line)) {
    line = trim(line);
    if (!line.empty() && line[0] != '#') lines.push_back(line);
  }
  return lines;
}

static std::string json_escape(const std::string& s) {
  std::ostringstream o;
  for (unsigned char c : s) {
    switch (c) {
      case '\\': o << "\\\\"; break;
      case '"': o << "\\\""; break;
      case '\b': o << "\\b"; break;
      case '\f': o << "\\f"; break;
      case '\n': o << "\\n"; break;
      case '\r': o << "\\r"; break;
      case '\t': o << "\\t"; break;
      default:
        if (c < 0x20) {
          o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
        } else {
          o << static_cast<char>(c);
        }
    }
  }
  return o.str();
}

static bool write_jpeg_atomic(const std::string& path, const cv::Mat& image) {
  if (path.empty() || image.empty()) return false;
  const std::string temporary = path + ".tmp.jpg";
  if (!cv::imwrite(temporary, image)) return false;
  if (std::rename(temporary.c_str(), path.c_str()) != 0) {
    std::remove(temporary.c_str());
    return false;
  }
  return true;
}

class LatestJpegWriter {
 public:
  LatestJpegWriter() : worker_([this] { loop(); }) {}

  ~LatestJpegWriter() {
    {
      std::lock_guard<std::mutex> lock(mu_);
      stop_ = true;
    }
    cv_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

  LatestJpegWriter(const LatestJpegWriter&) = delete;
  LatestJpegWriter& operator=(const LatestJpegWriter&) = delete;

  void submit(const std::string& overlay_path, const cv::Mat& overlay,
              const std::string& crop_path, const cv::Mat& crop) {
    Pending next;
    next.overlay_path = overlay_path;
    next.crop_path = crop_path;
    if (!overlay_path.empty() && !overlay.empty()) next.overlay = overlay.clone();
    if (!crop_path.empty() && !crop.empty()) next.crop = crop.clone();

    {
      std::lock_guard<std::mutex> lock(mu_);
      if (pending_) ++dropped_;
      pending_frame_ = std::move(next);
      pending_ = true;
      ++submitted_;
    }
    cv_.notify_one();
  }

  uint64_t submitted() const { return submitted_.load(); }
  uint64_t written() const { return written_.load(); }
  uint64_t dropped() const { return dropped_.load(); }

 private:
  struct Pending {
    std::string overlay_path;
    std::string crop_path;
    cv::Mat overlay;
    cv::Mat crop;
  };

  void loop() {
    while (true) {
      Pending current;
      {
        std::unique_lock<std::mutex> lock(mu_);
        cv_.wait(lock, [&] { return stop_ || pending_; });
        if (!pending_ && stop_) break;
        current = std::move(pending_frame_);
        pending_ = false;
      }

      if (!current.overlay_path.empty()) write_jpeg_atomic(current.overlay_path, current.overlay);
      if (!current.crop_path.empty()) write_jpeg_atomic(current.crop_path, current.crop);
      ++written_;
    }
  }

  std::thread worker_;
  mutable std::mutex mu_;
  std::condition_variable cv_;
  Pending pending_frame_;
  bool pending_{false};
  bool stop_{false};
  std::atomic<uint64_t> submitted_{0};
  std::atomic<uint64_t> written_{0};
  std::atomic<uint64_t> dropped_{0};
};

static size_t dtype_size(nvinfer1::DataType t) {
  switch (t) {
    case nvinfer1::DataType::kFLOAT: return 4;
    case nvinfer1::DataType::kHALF: return 2;
    case nvinfer1::DataType::kINT8: return 1;
    case nvinfer1::DataType::kINT32: return 4;
    case nvinfer1::DataType::kBOOL: return 1;
    case nvinfer1::DataType::kUINT8: return 1;
    default: return 4;
  }
}

static size_t volume(const nvinfer1::Dims& d) {
  size_t v = 1;
  for (int i = 0; i < d.nbDims; ++i) v *= static_cast<size_t>(std::max<int64_t>(1, d.d[i]));
  return v;
}

class TrtEngine {
 public:
  TrtEngine() = default;

  TrtEngine(const std::string& path, Logger& logger) { load(path, logger); }

  ~TrtEngine() { release(); }

  TrtEngine(const TrtEngine&) = delete;
  TrtEngine& operator=(const TrtEngine&) = delete;

  void load(const std::string& path, Logger& logger) {
    path_ = path;
    auto bytes = read_file_binary(path);
    runtime_ = nvinfer1::createInferRuntime(logger);
    engine_ = runtime_->deserializeCudaEngine(bytes.data(), bytes.size());
    if (!engine_) throw std::runtime_error("deserializeCudaEngine failed: " + path);
    context_ = engine_->createExecutionContext();
    if (!context_) throw std::runtime_error("createExecutionContext failed: " + path);

    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char* name = engine_->getIOTensorName(i);
      nvinfer1::Dims dims = engine_->getTensorShape(name);
      nvinfer1::DataType dtype = engine_->getTensorDataType(name);
      size_t nbytes = volume(dims) * dtype_size(dtype);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        input_name_ = name;
        input_dtype_ = dtype;
        input_dims_ = dims;
        input_bytes_ = nbytes;
        if (input_dtype_ != nvinfer1::DataType::kFLOAT) {
          throw std::runtime_error("only FP32 input tensor is supported for " + path);
        }
        check_cuda(cudaMalloc(&input_dev_, nbytes), "TRT input dev");
        context_->setTensorAddress(name, input_dev_);
      } else {
        Output out;
        out.name = name;
        out.dtype = dtype;
        out.dims = dims;
        out.bytes = nbytes;
        check_cuda(cudaHostAlloc(&out.host, nbytes, cudaHostAllocDefault), "TRT output host");
        check_cuda(cudaMalloc(&out.dev, nbytes), "TRT output dev");
        context_->setTensorAddress(name, out.dev);
        outputs_.push_back(out);
      }
    }
    if (input_name_.empty() || outputs_.empty()) throw std::runtime_error("engine IO discovery failed: " + path);
  }

  void release() {
    if (input_dev_) cudaFree(input_dev_);
    input_dev_ = nullptr;
    for (auto& o : outputs_) {
      if (o.dev) cudaFree(o.dev);
      if (o.host) cudaFreeHost(o.host);
    }
    outputs_.clear();
    delete context_;
    delete engine_;
    delete runtime_;
    context_ = nullptr;
    engine_ = nullptr;
    runtime_ = nullptr;
  }

  double run(cudaStream_t stream, const float* input_host) {
    auto t0 = Clock::now();
    check_cuda(cudaMemcpyAsync(input_dev_, input_host, input_bytes_, cudaMemcpyHostToDevice, stream), "TRT H2D");
    if (!context_->enqueueV3(stream)) throw std::runtime_error("TensorRT enqueueV3 failed: " + path_);
    for (auto& o : outputs_) {
      check_cuda(cudaMemcpyAsync(o.host, o.dev, o.bytes, cudaMemcpyDeviceToHost, stream), "TRT D2H");
    }
    check_cuda(cudaStreamSynchronize(stream), "TRT sync");
    auto t1 = Clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
  }

  std::vector<float> output_as_float(size_t idx = 0) const {
    if (idx >= outputs_.size()) return {};
    const auto& o = outputs_[idx];
    size_t n = output_count(idx);
    std::vector<float> y(n);
    if (o.dtype == nvinfer1::DataType::kFLOAT) {
      std::memcpy(y.data(), o.host, n * sizeof(float));
    } else if (o.dtype == nvinfer1::DataType::kHALF) {
      const auto* h = static_cast<const __half*>(o.host);
      for (size_t i = 0; i < n; ++i) y[i] = __half2float(h[i]);
    } else if (o.dtype == nvinfer1::DataType::kINT32) {
      const auto* v = static_cast<const int32_t*>(o.host);
      for (size_t i = 0; i < n; ++i) y[i] = static_cast<float>(v[i]);
    } else {
      const auto* v = static_cast<const uint8_t*>(o.host);
      for (size_t i = 0; i < n; ++i) y[i] = static_cast<float>(v[i]);
    }
    return y;
  }

  size_t input_bytes() const { return input_bytes_; }
  int input_h() const { return input_dims_.nbDims >= 4 ? static_cast<int>(input_dims_.d[input_dims_.nbDims - 2]) : 0; }
  int input_w() const { return input_dims_.nbDims >= 4 ? static_cast<int>(input_dims_.d[input_dims_.nbDims - 1]) : 0; }
  size_t output_count(size_t idx = 0) const { return outputs_[idx].bytes / dtype_size(outputs_[idx].dtype); }

 private:
  struct Output {
    std::string name;
    nvinfer1::DataType dtype{};
    nvinfer1::Dims dims{};
    size_t bytes{};
    void* host{};
    void* dev{};
  };

  std::string path_;
  nvinfer1::IRuntime* runtime_{};
  nvinfer1::ICudaEngine* engine_{};
  nvinfer1::IExecutionContext* context_{};
  std::string input_name_;
  nvinfer1::DataType input_dtype_{};
  nvinfer1::Dims input_dims_{};
  size_t input_bytes_{};
  void* input_dev_{};
  std::vector<Output> outputs_;
};

struct Config {
  std::string source;
  std::string det_engine;
  std::string image_cls_engine;
  std::string digit_cls_engine;
  std::string image_labels;
  std::string digit_labels;
  std::string classification_mode{"image"};
  std::string classification_mode_file;
  int det_width{1536};
  int det_height{1536};
  int image_cls_size{640};
  int digit_cls_size{128};
  float conf_threshold{0.25f};
  float roi_expand{1.05f};
  float keypoint_threshold{0.25f};
  float square_scale{1.0f};
  int canonical_crop_size{128};
  int max_detections{20};
  float nms_iou{0.45f};
  float nms_keypoint_distance{0.12f};
  bool fallback_center_crop{false};
  double warmup_sec{3.0};
  double measure_sec{0.0};
  int print_every{30};
  int caps_w{0};
  int caps_h{0};
  int mvs_index{0};
  float mvs_fps{60.0f};
  float mvs_exposure_us{2500.0f};
  float mvs_gain{-1.0f};
  bool async_capture{true};
  std::string overlay_jpeg;
  std::string crop_jpeg;
  int overlay_every{5};
  bool overlay_draw{false};
};

static void set_config_value(Config& c, const std::string& key, const std::string& value) {
  const std::string k = lower(trim(key));
  const std::string v = trim(value);
  if (k == "source") c.source = v;
  else if (k == "det_engine") c.det_engine = v;
  else if (k == "image_cls_engine") c.image_cls_engine = v;
  else if (k == "digit_cls_engine") c.digit_cls_engine = v;
  else if (k == "image_labels") c.image_labels = v;
  else if (k == "digit_labels") c.digit_labels = v;
  else if (k == "classification_mode") c.classification_mode = lower(v);
  else if (k == "classification_mode_file") c.classification_mode_file = v;
  else if (k == "det_size") c.det_width = c.det_height = std::atoi(v.c_str());
  else if (k == "det_width") c.det_width = std::atoi(v.c_str());
  else if (k == "det_height") c.det_height = std::atoi(v.c_str());
  else if (k == "image_cls_size") c.image_cls_size = std::atoi(v.c_str());
  else if (k == "digit_cls_size") c.digit_cls_size = std::atoi(v.c_str());
  else if (k == "conf_threshold") c.conf_threshold = std::atof(v.c_str());
  else if (k == "roi_expand") c.roi_expand = std::atof(v.c_str());
  else if (k == "keypoint_threshold") c.keypoint_threshold = std::atof(v.c_str());
  else if (k == "square_scale") c.square_scale = std::atof(v.c_str());
  else if (k == "canonical_crop_size") c.canonical_crop_size = std::atoi(v.c_str());
  else if (k == "max_detections") c.max_detections = std::atoi(v.c_str());
  else if (k == "nms_iou") c.nms_iou = std::atof(v.c_str());
  else if (k == "nms_keypoint_distance") c.nms_keypoint_distance = std::atof(v.c_str());
  else if (k == "fallback_center_crop") c.fallback_center_crop = lower(v) == "true" || v == "1";
  else if (k == "warmup_sec") c.warmup_sec = std::atof(v.c_str());
  else if (k == "measure_sec") c.measure_sec = std::atof(v.c_str());
  else if (k == "print_every") c.print_every = std::atoi(v.c_str());
  else if (k == "caps_w") c.caps_w = std::atoi(v.c_str());
  else if (k == "caps_h") c.caps_h = std::atoi(v.c_str());
  else if (k == "mvs_index") c.mvs_index = std::atoi(v.c_str());
  else if (k == "mvs_fps") c.mvs_fps = std::atof(v.c_str());
  else if (k == "mvs_exposure_us") c.mvs_exposure_us = std::atof(v.c_str());
  else if (k == "mvs_gain") c.mvs_gain = std::atof(v.c_str());
  else if (k == "async_capture") c.async_capture = lower(v) == "true" || v == "1";
  else if (k == "overlay_jpeg") c.overlay_jpeg = v;
  else if (k == "crop_jpeg") c.crop_jpeg = v;
  else if (k == "overlay_every") c.overlay_every = std::atoi(v.c_str());
  else if (k == "overlay_draw") c.overlay_draw = lower(v) == "true" || v == "1";
}

static Config load_config_file(const std::string& path) {
  Config c;
  std::ifstream f(path);
  if (!f) return c;
  std::string line;
  while (std::getline(f, line)) {
    size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    size_t colon = line.find(':');
    if (colon == std::string::npos) continue;
    set_config_value(c, line.substr(0, colon), line.substr(colon + 1));
  }
  return c;
}

static Config parse_args(int argc, char** argv) {
  std::string config_path = "configs/youth_pipeline.yaml";
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--config" && i + 1 < argc) config_path = argv[++i];
  }
  Config c = load_config_file(config_path);
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const std::string& key) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + key);
      return argv[++i];
    };
    if (a == "--config") {
      ++i;
    } else if (a == "--source") {
      c.source = need(a);
    } else if (a == "--det-engine") {
      c.det_engine = need(a);
    } else if (a == "--image-engine") {
      c.image_cls_engine = need(a);
    } else if (a == "--digit-engine") {
      c.digit_cls_engine = need(a);
    } else if (a == "--class-mode") {
      c.classification_mode = lower(need(a));
    } else if (a == "--mode-file") {
      c.classification_mode_file = need(a);
    } else if (a == "--conf") {
      c.conf_threshold = std::atof(need(a).c_str());
    } else if (a == "--print-every") {
      c.print_every = std::atoi(need(a).c_str());
    } else if (a == "--fallback-center-crop") {
      c.fallback_center_crop = true;
    } else if (a == "--no-fallback-center-crop") {
      c.fallback_center_crop = false;
    } else if (a == "--roi-expand") {
      c.roi_expand = std::atof(need(a).c_str());
    } else if (a == "--keypoint-threshold") {
      c.keypoint_threshold = std::atof(need(a).c_str());
    } else if (a == "--square-scale") {
      c.square_scale = std::atof(need(a).c_str());
    } else if (a == "--max-detections") {
      c.max_detections = std::atoi(need(a).c_str());
    } else if (a == "--nms-iou") {
      c.nms_iou = std::atof(need(a).c_str());
    } else if (a == "--nms-keypoint-distance") {
      c.nms_keypoint_distance = std::atof(need(a).c_str());
    } else if (a == "--warmup") {
      c.warmup_sec = std::atof(need(a).c_str());
    } else if (a == "--measure") {
      c.measure_sec = std::atof(need(a).c_str());
    } else if (a == "--caps-w") {
      c.caps_w = std::atoi(need(a).c_str());
    } else if (a == "--caps-h") {
      c.caps_h = std::atoi(need(a).c_str());
    } else if (a == "--mvs-fps") {
      c.mvs_fps = std::atof(need(a).c_str());
    } else if (a == "--mvs-exposure-us") {
      c.mvs_exposure_us = std::atof(need(a).c_str());
    } else if (a == "--mvs-gain") {
      c.mvs_gain = std::atof(need(a).c_str());
    } else if (a == "--async-capture") {
      c.async_capture = true;
    } else if (a == "--sync-capture") {
      c.async_capture = false;
    } else if (a == "--overlay-jpeg") {
      c.overlay_jpeg = need(a);
      c.overlay_draw = true;
    } else if (a == "--crop-jpeg") {
      c.crop_jpeg = need(a);
    } else if (a == "--overlay-every") {
      c.overlay_every = std::atoi(need(a).c_str());
    } else if (a == "--overlay-draw") {
      c.overlay_draw = true;
    } else if (a == "--help") {
      std::cout << "usage: " << argv[0] << " --config configs/youth_pipeline.yaml [--source video.mp4] [--class-mode image|digit]\n";
      std::exit(0);
    }
  }
  if (c.classification_mode != "image" && c.classification_mode != "digit") {
    throw std::runtime_error("classification_mode must be image or digit");
  }
  return c;
}

static cv::VideoCapture make_capture(const std::string& source, int caps_w, int caps_h) {
  if (starts_with(source, "gst:")) return cv::VideoCapture(source.substr(4), cv::CAP_GSTREAMER);
  bool numeric = !source.empty() && std::all_of(source.begin(), source.end(), [](unsigned char c) { return std::isdigit(c); });
  if (numeric) return cv::VideoCapture(std::atoi(source.c_str()));

  std::string caps = "video/x-raw,format=BGRx";
  if (caps_w > 0 && caps_h > 0) caps += ",width=" + std::to_string(caps_w) + ",height=" + std::to_string(caps_h);
  std::string pipe =
      "filesrc location=" + source +
      " ! qtdemux ! h264parse ! nvv4l2decoder enable-max-performance=1"
      " ! nvvidconv ! " + caps +
      " ! videoconvert ! video/x-raw,format=BGR"
      " ! appsink drop=true sync=false max-buffers=2";
  cv::VideoCapture cap(pipe, cv::CAP_GSTREAMER);
  if (!cap.isOpened()) cap.open(source);
  return cap;
}

static std::string mvs_chars_to_string(const unsigned char* s, size_t n) {
  std::string out;
  for (size_t i = 0; i < n && s[i]; ++i) out.push_back(static_cast<char>(s[i]));
  return out;
}

static void check_mvs(int ret, const char* what) {
  if (ret != MV_OK) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s failed: 0x%x", what, ret);
    throw std::runtime_error(buf);
  }
}

class MvsCapture {
 public:
  MvsCapture(int index, float fps, float exposure_us, float gain) {
    check_mvs(MV_CC_Initialize(), "MV_CC_Initialize");
    initialized_ = true;
    MV_CC_DEVICE_INFO_LIST list{};
    check_mvs(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list), "MV_CC_EnumDevices");
    if (list.nDeviceNum <= static_cast<unsigned int>(index)) {
      throw std::runtime_error("MVS camera index out of range; found " + std::to_string(list.nDeviceNum));
    }
    auto* info = list.pDeviceInfo[index];
    if (info->nTLayerType == MV_USB_DEVICE) {
      std::cerr << "[MVS] USB3Vision model="
                << mvs_chars_to_string(info->SpecialInfo.stUsb3VInfo.chModelName, sizeof(info->SpecialInfo.stUsb3VInfo.chModelName))
                << " serial="
                << mvs_chars_to_string(info->SpecialInfo.stUsb3VInfo.chSerialNumber, sizeof(info->SpecialInfo.stUsb3VInfo.chSerialNumber))
                << "\n";
    }
    check_mvs(MV_CC_CreateHandle(&handle_, info), "MV_CC_CreateHandle");
    check_mvs(MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0), "MV_CC_OpenDevice");
    opened_ = true;
    MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
    MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", fps);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
    MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_us);
    if (gain >= 0.0f) {
      MV_CC_SetEnumValue(handle_, "GainAuto", 0);
      MV_CC_SetFloatValue(handle_, "Gain", gain);
    }
    MV_CC_SetImageNodeNum(handle_, 4);
    check_mvs(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");
    grabbing_ = true;
  }

  ~MvsCapture() { close(); }

  MvsCapture(const MvsCapture&) = delete;
  MvsCapture& operator=(const MvsCapture&) = delete;

  bool read(cv::Mat& out) {
    if (!handle_) return false;
    MV_FRAME_OUT frame{};
    int ret = MV_CC_GetImageBuffer(handle_, &frame, 1000);
    if (ret != MV_OK) {
      std::cerr << "[MVS] GetImageBuffer failed: 0x" << std::hex << ret << std::dec << "\n";
      return false;
    }
    int width = static_cast<int>(frame.stFrameInfo.nWidth);
    int height = static_cast<int>(frame.stFrameInfo.nHeight);
    size_t needed = static_cast<size_t>(width) * height * 3;
    if (bgr_.size() != needed) bgr_.resize(needed);

    MV_CC_PIXEL_CONVERT_PARAM conv{};
    conv.nWidth = frame.stFrameInfo.nWidth;
    conv.nHeight = frame.stFrameInfo.nHeight;
    conv.pSrcData = frame.pBufAddr;
    conv.nSrcDataLen = frame.stFrameInfo.nFrameLen;
    conv.enSrcPixelType = frame.stFrameInfo.enPixelType;
    conv.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    conv.pDstBuffer = bgr_.data();
    conv.nDstBufferSize = static_cast<unsigned int>(bgr_.size());
    ret = MV_CC_ConvertPixelType(handle_, &conv);
    MV_CC_FreeImageBuffer(handle_, &frame);
    if (ret != MV_OK) {
      std::cerr << "[MVS] ConvertPixelType failed: 0x" << std::hex << ret << std::dec << "\n";
      return false;
    }
    out = cv::Mat(height, width, CV_8UC3, bgr_.data());
    return true;
  }

 private:
  void close() {
    if (handle_) {
      if (grabbing_) MV_CC_StopGrabbing(handle_);
      if (opened_) MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
      handle_ = nullptr;
    }
    if (initialized_) MV_CC_Finalize();
    initialized_ = false;
  }

  void* handle_{};
  bool initialized_{};
  bool opened_{};
  bool grabbing_{};
  std::vector<unsigned char> bgr_;
};

class AsyncMvsCapture {
 public:
  AsyncMvsCapture(int index, float fps, float exposure_us, float gain)
      : cap_(index, fps, exposure_us, gain), worker_([this] { loop(); }) {}

  ~AsyncMvsCapture() {
    stop_.store(true);
    frame_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

  AsyncMvsCapture(const AsyncMvsCapture&) = delete;
  AsyncMvsCapture& operator=(const AsyncMvsCapture&) = delete;

  bool read(cv::Mat& out) {
    std::unique_lock<std::mutex> lock(mu_);
    bool ok = frame_cv_.wait_for(lock, std::chrono::milliseconds(1000), [&] {
      return stop_.load() || seq_ != last_seq_;
    });
    if (!ok || !latest_) return false;
    out = *latest_;
    last_seq_ = seq_;
    return true;
  }

 private:
  void loop() {
    cv::Mat tmp;
    while (!stop_.load()) {
      if (!cap_.read(tmp) || tmp.empty()) continue;
      auto frame = std::make_shared<cv::Mat>(tmp.clone());
      {
        std::lock_guard<std::mutex> lock(mu_);
        latest_ = std::move(frame);
        ++seq_;
      }
      frame_cv_.notify_one();
    }
  }

  MvsCapture cap_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
  std::mutex mu_;
  std::condition_variable frame_cv_;
  std::shared_ptr<cv::Mat> latest_;
  uint64_t seq_{0};
  uint64_t last_seq_{0};
};

struct LetterboxMeta {
  double scale{1.0};
  double pad_x{0.0};
  double pad_y{0.0};
  int src_w{0};
  int src_h{0};
};

class LetterboxPreprocessor {
 public:
  LetterboxPreprocessor(int target_width, int target_height)
      : target_width_(target_width), target_height_(target_height) {
    canvas_.create(target_height_, target_width_, CV_8UC3);
    rgb_.create(target_height_, target_width_, CV_8UC3);
    f32_.create(target_height_, target_width_, CV_32FC3);
  }

  LetterboxMeta run(const cv::Mat& src, float* dst) {
    LetterboxMeta m;
    m.src_w = src.cols;
    m.src_h = src.rows;
    canvas_.setTo(cv::Scalar(114, 114, 114));
    m.scale = std::min(static_cast<double>(target_width_) / src.cols,
                       static_cast<double>(target_height_) / src.rows);
    int nw = std::max(1, static_cast<int>(std::round(src.cols * m.scale)));
    int nh = std::max(1, static_cast<int>(std::round(src.rows * m.scale)));
    int x = (target_width_ - nw) / 2;
    int y = (target_height_ - nh) / 2;
    m.pad_x = x;
    m.pad_y = y;
    resized_.create(nh, nw, CV_8UC3);
    cv::resize(src, resized_, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    resized_.copyTo(canvas_(cv::Rect(x, y, nw, nh)));
    cv::cvtColor(canvas_, rgb_, cv::COLOR_BGR2RGB);
    rgb_.convertTo(f32_, CV_32F, 1.0 / 255.0);
    const size_t plane = static_cast<size_t>(target_width_) * target_height_;
    cv::Mat chw[] = {
        cv::Mat(target_height_, target_width_, CV_32F, dst),
        cv::Mat(target_height_, target_width_, CV_32F, dst + plane),
        cv::Mat(target_height_, target_width_, CV_32F, dst + 2 * plane),
    };
    cv::split(f32_, chw);
    return m;
  }

 private:
  int target_width_;
  int target_height_;
  cv::Mat canvas_, resized_, rgb_, f32_;
};

class ClassifyPreprocessor {
 public:
  explicit ClassifyPreprocessor(int target) : target_(target) {
    crop_.create(target_, target_, CV_8UC3);
    rgb_.create(target_, target_, CV_8UC3);
    f32_.create(target_, target_, CV_32FC3);
  }

  void run(const cv::Mat& roi, float* dst) {
    double scale = static_cast<double>(target_) / std::min(roi.cols, roi.rows);
    int nw = std::max(target_, static_cast<int>(std::round(roi.cols * scale)));
    int nh = std::max(target_, static_cast<int>(std::round(roi.rows * scale)));
    resized_.create(nh, nw, CV_8UC3);
    cv::resize(roi, resized_, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    int x = std::max(0, (nw - target_) / 2);
    int y = std::max(0, (nh - target_) / 2);
    resized_(cv::Rect(x, y, target_, target_)).copyTo(crop_);
    cv::cvtColor(crop_, rgb_, cv::COLOR_BGR2RGB);
    rgb_.convertTo(f32_, CV_32F, 1.0 / 255.0);
    cv::Mat chw[] = {
        cv::Mat(target_, target_, CV_32F, dst),
        cv::Mat(target_, target_, CV_32F, dst + target_ * target_),
        cv::Mat(target_, target_, CV_32F, dst + 2 * target_ * target_),
    };
    cv::split(f32_, chw);
  }

 private:
  int target_;
  cv::Mat resized_, crop_, rgb_, f32_;
};

struct Detection {
  bool found{false};
  float score{}, cls{};
  cv::Rect roi;
  std::vector<cv::Point> corners;
  std::array<cv::Point2f, 3> keypoints{};
  std::array<float, 3> keypoint_scores{};
  std::array<cv::Point2f, 4> class_quad{};
  bool square_valid{false};
};

static cv::Point2f map_canvas_to_src(const cv::Point2f& p, const LetterboxMeta& m) {
  return cv::Point2f(static_cast<float>((p.x - m.pad_x) / m.scale), static_cast<float>((p.y - m.pad_y) / m.scale));
}

static cv::Point2f clamp_to_src(const cv::Point2f& p, const LetterboxMeta& meta) {
  return cv::Point2f(
      std::clamp(p.x, 0.0f, static_cast<float>(std::max(0, meta.src_w - 1))),
      std::clamp(p.y, 0.0f, static_cast<float>(std::max(0, meta.src_h - 1))));
}

static bool build_directional_square(Detection& d, float square_scale) {
  const cv::Point2f tip = d.keypoints[0];
  const cv::Point2f base_left = d.keypoints[1];
  const cv::Point2f base_right = d.keypoints[2];
  const cv::Point2f edge = base_right - base_left;
  const float base_length = std::sqrt(edge.dot(edge));
  if (base_length < 4.0f || square_scale <= 0.0f) return false;

  const cv::Point2f base_mid = (base_left + base_right) * 0.5f;
  const cv::Point2f toward_tip = tip - base_mid;
  if (std::sqrt(toward_tip.dot(toward_tip)) < 4.0f) return false;

  const cv::Point2f tangent = edge * (1.0f / base_length);
  cv::Point2f normal(-tangent.y, tangent.x);
  if (normal.dot(toward_tip) < 0.0f) normal *= -1.0f;

  const float side = base_length * square_scale;
  const cv::Point2f lower_left = base_mid - tangent * (side * 0.5f);
  const cv::Point2f lower_right = base_mid + tangent * (side * 0.5f);
  const cv::Point2f upper_left = lower_left + normal * side;
  const cv::Point2f upper_right = lower_right + normal * side;
  d.class_quad = {upper_left, upper_right, lower_right, lower_left};
  d.square_valid = true;
  return true;
}

static bool fill_pose_geometry(Detection& d, const float* row, const LetterboxMeta& meta,
                               float keypoint_threshold, float square_scale) {
  const cv::Point2f box_a = clamp_to_src(map_canvas_to_src(cv::Point2f(row[0], row[1]), meta), meta);
  const cv::Point2f box_b = clamp_to_src(map_canvas_to_src(cv::Point2f(row[2], row[3]), meta), meta);
  const int x1 = std::clamp(static_cast<int>(std::floor(std::min(box_a.x, box_b.x))), 0, std::max(0, meta.src_w - 1));
  const int y1 = std::clamp(static_cast<int>(std::floor(std::min(box_a.y, box_b.y))), 0, std::max(0, meta.src_h - 1));
  const int x2 = std::clamp(static_cast<int>(std::ceil(std::max(box_a.x, box_b.x))), 0, meta.src_w);
  const int y2 = std::clamp(static_cast<int>(std::ceil(std::max(box_a.y, box_b.y))), 0, meta.src_h);
  if (x2 <= x1 + 1 || y2 <= y1 + 1) return false;

  d.roi = cv::Rect(x1, y1, x2 - x1, y2 - y1);
  d.corners = {
      cv::Point(x1, y1),
      cv::Point(x2 - 1, y1),
      cv::Point(x2 - 1, y2 - 1),
      cv::Point(x1, y2 - 1),
  };

  bool keypoints_valid = true;
  for (size_t k = 0; k < d.keypoints.size(); ++k) {
    const size_t offset = 6 + k * 3;
    d.keypoints[k] = clamp_to_src(map_canvas_to_src(cv::Point2f(row[offset], row[offset + 1]), meta), meta);
    d.keypoint_scores[k] = row[offset + 2];
    keypoints_valid = keypoints_valid && d.keypoint_scores[k] >= keypoint_threshold;
  }
  return keypoints_valid && build_directional_square(d, square_scale);
}

static float rect_iou(const cv::Rect& a, const cv::Rect& b) {
  int inter = (a & b).area();
  if (inter <= 0) return 0.0f;
  int uni = a.area() + b.area() - inter;
  return uni > 0 ? static_cast<float>(inter) / static_cast<float>(uni) : 0.0f;
}

static float point_distance(const cv::Point2f& a, const cv::Point2f& b) {
  const cv::Point2f delta = a - b;
  return std::sqrt(delta.dot(delta));
}

static float pose_extent(const Detection& d) {
  const float base = point_distance(d.keypoints[1], d.keypoints[2]);
  const cv::Point2f base_mid = (d.keypoints[1] + d.keypoints[2]) * 0.5f;
  const float height = point_distance(d.keypoints[0], base_mid);
  return std::max(4.0f, std::max(base, height));
}

static float normalized_pose_distance(const Detection& a, const Detection& b) {
  const float scale = std::max(4.0f, 0.5f * (pose_extent(a) + pose_extent(b)));
  const float tip_sq = std::pow(point_distance(a.keypoints[0], b.keypoints[0]), 2.0f);
  const float direct_sq = tip_sq
      + std::pow(point_distance(a.keypoints[1], b.keypoints[1]), 2.0f)
      + std::pow(point_distance(a.keypoints[2], b.keypoints[2]), 2.0f);
  const float swapped_sq = tip_sq
      + std::pow(point_distance(a.keypoints[1], b.keypoints[2]), 2.0f)
      + std::pow(point_distance(a.keypoints[2], b.keypoints[1]), 2.0f);
  return std::sqrt(std::min(direct_sq, swapped_sq) / 3.0f) / scale;
}

static float detection_rank_score(const Detection& d) {
  const float keypoint_mean = std::accumulate(d.keypoint_scores.begin(), d.keypoint_scores.end(), 0.0f)
      / static_cast<float>(d.keypoint_scores.size());
  return d.score * keypoint_mean;
}

static bool is_pose_duplicate(const Detection& candidate, const Detection& kept,
                              float nms_iou, float nms_keypoint_distance) {
  const float iou = rect_iou(candidate.roi, kept.roi);
  if (nms_keypoint_distance <= 0.0f) return iou > nms_iou;

  const float pose_distance = normalized_pose_distance(candidate, kept);
  if (pose_distance <= nms_keypoint_distance) return true;

  const float relaxed_pose_distance = std::max(
      nms_keypoint_distance + 0.08f, nms_keypoint_distance * 1.75f);
  if (iou > nms_iou && pose_distance <= relaxed_pose_distance) return true;

  return iou > 0.85f;
}

struct PoseParseStats {
  size_t rows{0};
  size_t score_or_box_rejected{0};
  size_t geometry_rejected{0};
  size_t candidates{0};
  size_t duplicates_suppressed{0};
};

static std::vector<Detection> parse_pose_detections(const std::vector<float>& out, const LetterboxMeta& meta, float conf,
                                                    float keypoint_threshold, float square_scale,
                                                    int max_detections, float nms_iou,
                                                    float nms_keypoint_distance, PoseParseStats* stats = nullptr) {
  constexpr size_t kPoseRow = 15;
  if (out.size() % kPoseRow != 0) {
    throw std::runtime_error("pose output must contain 15 values per detection");
  }

  PoseParseStats local_stats;
  local_stats.rows = out.size() / kPoseRow;
  std::vector<Detection> candidates;
  candidates.reserve(local_stats.rows);
  for (size_t i = 0; i + kPoseRow <= out.size(); i += kPoseRow) {
    float score = out[i + 4];
    if (score < conf || out[i + 2] <= out[i] + 2.0f || out[i + 3] <= out[i + 1] + 2.0f) {
      ++local_stats.score_or_box_rejected;
      continue;
    }

    Detection d;
    d.found = true;
    d.score = score;
    d.cls = out[i + 5];
    if (fill_pose_geometry(d, out.data() + i, meta, keypoint_threshold, square_scale)) {
      candidates.push_back(std::move(d));
    } else {
      ++local_stats.geometry_rejected;
    }
  }
  local_stats.candidates = candidates.size();

  std::sort(candidates.begin(), candidates.end(), [](const Detection& a, const Detection& b) {
    return detection_rank_score(a) > detection_rank_score(b);
  });

  std::vector<Detection> selected;
  selected.reserve(max_detections > 0 ? std::min(candidates.size(), static_cast<size_t>(max_detections)) : candidates.size());
  for (const auto& d : candidates) {
    bool duplicate = false;
    for (const auto& kept : selected) {
      if (is_pose_duplicate(d, kept, nms_iou, nms_keypoint_distance)) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      ++local_stats.duplicates_suppressed;
      continue;
    }
    selected.push_back(d);
    if (max_detections > 0 && static_cast<int>(selected.size()) >= max_detections) break;
  }
  if (stats) *stats = local_stats;
  return selected;
}

static void append_pose_test_row(std::vector<float>& out, float cx, float cy, float score,
                                 float keypoint_score, float shift = 0.0f, bool swap_base = false) {
  const cv::Point2f tip(cx + shift, cy - 40.0f);
  cv::Point2f base_left(cx - 30.0f + shift, cy + 40.0f);
  cv::Point2f base_right(cx + 30.0f + shift, cy + 40.0f);
  if (swap_base) std::swap(base_left, base_right);
  const std::array<float, 15> row = {
      cx - 40.0f + shift, cy - 50.0f, cx + 40.0f + shift, cy + 50.0f,
      score, 0.0f,
      tip.x, tip.y, keypoint_score,
      base_left.x, base_left.y, keypoint_score,
      base_right.x, base_right.y, keypoint_score,
  };
  out.insert(out.end(), row.begin(), row.end());
}

static int run_pose_nms_self_test() {
  LetterboxMeta meta;
  meta.src_w = 512;
  meta.src_h = 512;
  std::vector<float> rows;
  append_pose_test_row(rows, 100.0f, 120.0f, 0.95f, 0.99f);
  append_pose_test_row(rows, 100.0f, 120.0f, 0.80f, 0.98f, 3.0f);
  append_pose_test_row(rows, 100.0f, 120.0f, 0.75f, 0.97f, 0.0f, true);
  append_pose_test_row(rows, 130.0f, 120.0f, 0.90f, 0.99f);
  append_pose_test_row(rows, 160.0f, 120.0f, 0.85f, 0.99f);
  append_pose_test_row(rows, 260.0f, 120.0f, 0.70f, 0.10f);

  PoseParseStats stats;
  const std::vector<Detection> selected = parse_pose_detections(
      rows, meta, 0.25f, 0.25f, 1.0f, 6, 0.45f, 0.12f, &stats);
  const bool passed = selected.size() == 3
      && stats.candidates == 5
      && stats.duplicates_suppressed == 2
      && stats.geometry_rejected == 1;
  std::cout << "{\"self_test\":\"pose_nms\",\"passed\":" << (passed ? "true" : "false")
            << ",\"selected\":" << selected.size()
            << ",\"candidates\":" << stats.candidates
            << ",\"suppressed\":" << stats.duplicates_suppressed
            << ",\"geometry_rejected\":" << stats.geometry_rejected << "}" << std::endl;
  return passed ? 0 : 3;
}

static bool extract_directional_square(const cv::Mat& frame, const Detection& d, int output_size, cv::Mat& crop) {
  if (!d.square_valid || output_size <= 1) return false;
  const float last = static_cast<float>(output_size - 1);
  const std::array<cv::Point2f, 4> destination = {
      cv::Point2f(0.0f, 0.0f),
      cv::Point2f(last, 0.0f),
      cv::Point2f(last, last),
      cv::Point2f(0.0f, last),
  };
  cv::Mat transform = cv::getPerspectiveTransform(d.class_quad.data(), destination.data());
  cv::warpPerspective(frame, crop, transform, cv::Size(output_size, output_size), cv::INTER_LINEAR,
                      cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  return !crop.empty();
}

struct ClassResult {
  int id{-1};
  float prob{0.0f};
  std::string label;
};

static bool is_ascii_overlay_text(const std::string& s) {
  return std::all_of(s.begin(), s.end(), [](unsigned char c) { return c >= 32 && c < 127; });
}

static std::string overlay_class_label(const ClassResult& cr) {
  if (cr.id < 0) return "";
  if (!cr.label.empty() && is_ascii_overlay_text(cr.label)) return cr.label;
  return "#" + std::to_string(cr.id);
}

static void draw_overlay(cv::Mat& frame, const std::vector<Detection>& detections, const std::vector<ClassResult>& classes,
                         const std::string& mode, double fps, double pre_ms, double det_ms, double cls_ms) {
  const cv::Scalar green(40, 255, 40);
  const cv::Scalar yellow(0, 200, 255);
  const cv::Scalar cyan(255, 240, 0);
  const std::array<cv::Scalar, 3> point_colors = {
      cv::Scalar(30, 30, 255),
      cv::Scalar(40, 230, 40),
      cv::Scalar(255, 120, 20),
  };
  for (size_t idx = 0; idx < detections.size(); ++idx) {
    const Detection& d = detections[idx];
    if (d.found && d.corners.size() == 4) {
      for (int i = 0; i < 4; ++i) cv::line(frame, d.corners[i], d.corners[(i + 1) % 4], green, 2, cv::LINE_AA);
      std::ostringstream ss;
      ss << "POSE#" << (idx + 1) << " " << std::fixed << std::setprecision(2) << d.score;
      cv::putText(frame, ss.str(), d.corners[0] + cv::Point(4, -6), cv::FONT_HERSHEY_SIMPLEX, 0.6, green, 2, cv::LINE_AA);
      for (size_t k = 0; k < d.keypoints.size(); ++k) {
        const cv::Point point(static_cast<int>(std::round(d.keypoints[k].x)),
                              static_cast<int>(std::round(d.keypoints[k].y)));
        cv::circle(frame, point, 5, point_colors[k], cv::FILLED, cv::LINE_AA);
        cv::circle(frame, point, 8, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      }
      const cv::Point base_mid(
          static_cast<int>(std::round((d.keypoints[1].x + d.keypoints[2].x) * 0.5f)),
          static_cast<int>(std::round((d.keypoints[1].y + d.keypoints[2].y) * 0.5f)));
      const cv::Point tip(static_cast<int>(std::round(d.keypoints[0].x)),
                          static_cast<int>(std::round(d.keypoints[0].y)));
      cv::arrowedLine(frame, base_mid, tip, cv::Scalar(255, 80, 255), 3, cv::LINE_AA, 0, 0.18);
      if (d.square_valid) {
        for (int i = 0; i < 4; ++i) {
          cv::line(frame, d.class_quad[i], d.class_quad[(i + 1) % 4], yellow, 2, cv::LINE_AA);
        }
      }
    }

    if (idx < classes.size() && classes[idx].id >= 0) {
      std::ostringstream ss;
      ss << mode << "#" << (idx + 1) << ": " << overlay_class_label(classes[idx]) << " "
         << std::fixed << std::setprecision(2) << classes[idx].prob;
      int y = d.roi.y + d.roi.height + 24;
      if (y >= frame.rows - 8) y = std::max(52, d.roi.y - 32);
      int x = std::max(8, d.roi.x);
      cv::putText(frame, ss.str(), cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.7, cyan, 2, cv::LINE_AA);
    }
  }

  std::ostringstream top;
  top << "FPS " << std::fixed << std::setprecision(1) << fps
      << " targets " << detections.size()
      << " pre " << std::setprecision(1) << pre_ms
      << " det " << det_ms
      << " cls " << cls_ms
      << " ms mode " << mode;
  cv::rectangle(frame, cv::Rect(0, 0, std::min(frame.cols, 880), 34), cv::Scalar(0, 0, 0), cv::FILLED);
  cv::putText(frame, top.str(), cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
}

static ClassResult classify_from_output(const std::vector<float>& scores, const std::vector<std::string>& labels) {
  ClassResult r;
  if (scores.empty()) return r;
  const auto max_it = std::max_element(scores.begin(), scores.end());
  r.id = static_cast<int>(max_it - scores.begin());

  double value_sum = 0.0;
  bool normalized_probabilities = true;
  for (float value : scores) {
    if (!std::isfinite(value) || value < -1e-4f || value > 1.0001f) normalized_probabilities = false;
    value_sum += value;
  }
  normalized_probabilities = normalized_probabilities && std::abs(value_sum - 1.0) <= 0.01;

  if (normalized_probabilities) {
    r.prob = std::clamp(*max_it, 0.0f, 1.0f);
  } else {
    const float maxv = *max_it;
    double exp_sum = 0.0;
    for (float value : scores) exp_sum += std::exp(static_cast<double>(value - maxv));
    r.prob = exp_sum > 0.0
        ? static_cast<float>(std::exp(static_cast<double>(scores[r.id] - maxv)) / exp_sum)
        : 0.0f;
  }
  r.label = (r.id >= 0 && static_cast<size_t>(r.id) < labels.size()) ? labels[r.id] : std::to_string(r.id);
  return r;
}

static std::string reload_mode_if_needed(const std::string& file, const std::string& current, Clock::time_point& last_check) {
  if (file.empty()) return current;
  auto now = Clock::now();
  if (std::chrono::duration<double>(now - last_check).count() < 1.0) return current;
  last_check = now;
  std::string text = lower(trim(read_text_file(file)));
  if (text == "image" || text == "digit") return text;
  return current;
}

int main(int argc, char** argv) {
  try {
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "--self-test-pose-nms") return run_pose_nms_self_test();
    }
    Config cfg = parse_args(argc, argv);
    Logger logger;
    TrtEngine det(cfg.det_engine, logger);
    TrtEngine image_cls(cfg.image_cls_engine, logger);
    TrtEngine digit_cls(cfg.digit_cls_engine, logger);
    std::vector<std::string> image_labels = read_lines(cfg.image_labels);
    std::vector<std::string> digit_labels = read_lines(cfg.digit_labels);

    if (det.input_w() != cfg.det_width || det.input_h() != cfg.det_height) {
      std::cerr << "warning: det engine input is " << det.input_w() << "x" << det.input_h()
                << ", config input is " << cfg.det_width << "x" << cfg.det_height << "\n";
      cfg.det_width = det.input_w();
      cfg.det_height = det.input_h();
    }
    if (image_cls.input_w() != cfg.image_cls_size || image_cls.input_h() != cfg.image_cls_size) {
      cfg.image_cls_size = image_cls.input_w();
    }
    if (digit_cls.input_w() != cfg.digit_cls_size || digit_cls.input_h() != cfg.digit_cls_size) {
      cfg.digit_cls_size = digit_cls.input_w();
    }
    if (det.output_count(0) % 15 != 0) {
      throw std::runtime_error("pose engine output is not Nx15");
    }
    if (cfg.canonical_crop_size != 128) {
      std::cerr << "warning: canonical crop is fixed to 128x128 for the directional square\n";
      cfg.canonical_crop_size = 128;
    }

    float* det_input{};
    float* image_input{};
    float* digit_input{};
    check_cuda(cudaHostAlloc(reinterpret_cast<void**>(&det_input), det.input_bytes(), cudaHostAllocDefault), "det input host");
    check_cuda(cudaHostAlloc(reinterpret_cast<void**>(&image_input), image_cls.input_bytes(), cudaHostAllocDefault), "image input host");
    check_cuda(cudaHostAlloc(reinterpret_cast<void**>(&digit_input), digit_cls.input_bytes(), cudaHostAllocDefault), "digit input host");
    cudaStream_t det_stream{}, cls_stream{};
    check_cuda(cudaStreamCreate(&det_stream), "det stream");
    check_cuda(cudaStreamCreate(&cls_stream), "cls stream");

    LetterboxPreprocessor det_pre(det.input_w(), det.input_h());
    ClassifyPreprocessor image_pre(cfg.image_cls_size);
    ClassifyPreprocessor digit_pre(cfg.digit_cls_size);

    const bool use_mvs = lower(cfg.source) == "mvs" || starts_with(lower(cfg.source), "mvs:");
    std::unique_ptr<MvsCapture> mvs_cap;
    std::unique_ptr<AsyncMvsCapture> async_mvs_cap;
    cv::VideoCapture cap;
    if (use_mvs) {
      if (cfg.async_capture) {
        async_mvs_cap.reset(new AsyncMvsCapture(cfg.mvs_index, cfg.mvs_fps, cfg.mvs_exposure_us, cfg.mvs_gain));
      } else {
        mvs_cap.reset(new MvsCapture(cfg.mvs_index, cfg.mvs_fps, cfg.mvs_exposure_us, cfg.mvs_gain));
      }
    } else {
      cap = make_capture(cfg.source, cfg.caps_w, cfg.caps_h);
      if (!cap.isOpened()) throw std::runtime_error("cannot open source: " + cfg.source);
    }

    std::string mode = cfg.classification_mode;
    Clock::time_point last_mode_check = Clock::now() - std::chrono::seconds(10);
    auto start = Clock::now();
    auto measure_start = start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(cfg.warmup_sec));
    int64_t frames_total = 0;
    int64_t frames_measured = 0;
    double det_ms_sum = 0.0, cls_ms_sum = 0.0, pre_ms_sum = 0.0;
    cv::Mat frame;
    LatestJpegWriter jpeg_writer;

    while (true) {
      auto loop_now = Clock::now();
      if (cfg.measure_sec > 0.0 && loop_now >= measure_start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(cfg.measure_sec))) break;

      if (use_mvs) {
        if (cfg.async_capture) {
          if (!async_mvs_cap->read(frame)) continue;
        } else if (!mvs_cap->read(frame)) {
          continue;
        }
      } else if (!cap.read(frame)) {
        cap.release();
        cap = make_capture(cfg.source, cfg.caps_w, cfg.caps_h);
        if (!cap.read(frame)) break;
      }
      if (frame.empty()) continue;
      ++frames_total;
      const bool emit_jpeg = cfg.overlay_every > 0 && frames_total % cfg.overlay_every == 0;
      mode = reload_mode_if_needed(cfg.classification_mode_file, mode, last_mode_check);

      auto p0 = Clock::now();
      LetterboxMeta meta = det_pre.run(frame, det_input);
      auto p1 = Clock::now();
      double det_ms = det.run(det_stream, det_input);
      std::vector<float> det_out = det.output_as_float(0);
      PoseParseStats pose_stats;
      std::vector<Detection> detections = parse_pose_detections(
          det_out, meta, cfg.conf_threshold, cfg.keypoint_threshold, cfg.square_scale,
          cfg.max_detections, cfg.nms_iou, cfg.nms_keypoint_distance, &pose_stats);

      std::vector<ClassResult> class_results(detections.size());
      cv::Mat crop_preview;
      double cls_ms = 0.0;
      for (size_t det_idx = 0; det_idx < detections.size(); ++det_idx) {
        const Detection& d = detections[det_idx];
        cv::Mat canonical_crop;
        if (extract_directional_square(frame, d, cfg.canonical_crop_size, canonical_crop)) {
          if (emit_jpeg && crop_preview.empty()) crop_preview = canonical_crop.clone();
          if (mode == "digit") {
            digit_pre.run(canonical_crop, digit_input);
            cls_ms += digit_cls.run(cls_stream, digit_input);
            class_results[det_idx] = classify_from_output(digit_cls.output_as_float(0), digit_labels);
          } else {
            image_pre.run(canonical_crop, image_input);
            cls_ms += image_cls.run(cls_stream, image_input);
            class_results[det_idx] = classify_from_output(image_cls.output_as_float(0), image_labels);
          }
        }
      }

      bool measured = Clock::now() >= measure_start;
      if (measured) {
        ++frames_measured;
        pre_ms_sum += std::chrono::duration<double, std::milli>(p1 - p0).count();
        det_ms_sum += det_ms;
        cls_ms_sum += cls_ms;
      }

      if (cfg.overlay_draw || (emit_jpeg && (!cfg.overlay_jpeg.empty() || !cfg.crop_jpeg.empty()))) {
        double elapsed = std::chrono::duration<double>(Clock::now() - measure_start).count();
        double fps = frames_measured > 0 && elapsed > 0.0 ? frames_measured / elapsed : 0.0;
        double pre_avg = frames_measured ? pre_ms_sum / frames_measured : 0.0;
        double det_avg = frames_measured ? det_ms_sum / frames_measured : 0.0;
        double cls_avg = frames_measured ? cls_ms_sum / frames_measured : 0.0;
        if (cfg.overlay_draw || (emit_jpeg && !cfg.overlay_jpeg.empty())) {
          draw_overlay(frame, detections, class_results, mode, fps, pre_avg, det_avg, cls_avg);
        }
        if (emit_jpeg) {
          if (!cfg.crop_jpeg.empty()) {
            if (crop_preview.empty()) {
              crop_preview = cv::Mat(cfg.canonical_crop_size, cfg.canonical_crop_size, CV_8UC3, cv::Scalar(20, 20, 20));
            }
          }
          jpeg_writer.submit(cfg.overlay_jpeg, frame, cfg.crop_jpeg, crop_preview);
        }
      }

      if (cfg.print_every > 0 && frames_total % cfg.print_every == 0) {
        double elapsed = std::chrono::duration<double>(Clock::now() - measure_start).count();
        double fps = frames_measured > 0 && elapsed > 0.0 ? frames_measured / elapsed : 0.0;
        const bool has_det = !detections.empty() && detections[0].found;
        const Detection first_det = !detections.empty() ? detections[0] : Detection{};
        const ClassResult first_cls = !class_results.empty() ? class_results[0] : ClassResult{};
        std::cout << "{"
                  << "\"frame\":" << frames_total << ","
                  << "\"fps\":" << fps << ","
                  << "\"mode\":\"" << mode << "\","
                  << "\"det_found\":" << (has_det ? "true" : "false") << ","
                  << "\"det_count\":" << detections.size() << ","
                  << "\"pose_candidates\":" << pose_stats.candidates << ","
                  << "\"pose_suppressed\":" << pose_stats.duplicates_suppressed << ","
                  << "\"pose_geometry_rejected\":" << pose_stats.geometry_rejected << ","
                  << "\"jpeg_dropped\":" << jpeg_writer.dropped() << ","
                  << "\"det_score\":" << first_det.score << ","
                  << "\"roi\":[" << first_det.roi.x << "," << first_det.roi.y << "," << first_det.roi.width << "," << first_det.roi.height << "],"
                  << "\"square_valid\":" << (first_det.square_valid ? "true" : "false") << ","
                  << "\"keypoints\":["
                  << "[" << first_det.keypoints[0].x << "," << first_det.keypoints[0].y << "," << first_det.keypoint_scores[0] << "],"
                  << "[" << first_det.keypoints[1].x << "," << first_det.keypoints[1].y << "," << first_det.keypoint_scores[1] << "],"
                  << "[" << first_det.keypoints[2].x << "," << first_det.keypoints[2].y << "," << first_det.keypoint_scores[2] << "]],"
                  << "\"class_id\":" << first_cls.id << ","
                  << "\"class_prob\":" << first_cls.prob << ","
                  << "\"class_label\":\"" << json_escape(first_cls.label) << "\","
                  << "\"detections\":[";
        for (size_t j = 0; j < detections.size(); ++j) {
          const Detection& item = detections[j];
          const ClassResult item_cls = j < class_results.size() ? class_results[j] : ClassResult{};
          if (j > 0) std::cout << ",";
          std::cout << "{"
                    << "\"score\":" << item.score << ","
                    << "\"roi\":[" << item.roi.x << "," << item.roi.y << "," << item.roi.width << "," << item.roi.height << "],"
                    << "\"square_valid\":" << (item.square_valid ? "true" : "false") << ","
                    << "\"keypoints\":["
                    << "[" << item.keypoints[0].x << "," << item.keypoints[0].y << "," << item.keypoint_scores[0] << "],"
                    << "[" << item.keypoints[1].x << "," << item.keypoints[1].y << "," << item.keypoint_scores[1] << "],"
                    << "[" << item.keypoints[2].x << "," << item.keypoints[2].y << "," << item.keypoint_scores[2] << "]],"
                    << "\"class_id\":" << item_cls.id << ","
                    << "\"class_prob\":" << item_cls.prob << ","
                    << "\"class_label\":\"" << json_escape(item_cls.label) << "\""
                    << "}";
        }
        std::cout << "],"
                  << "\"mean_pre_ms\":" << (frames_measured ? pre_ms_sum / frames_measured : 0.0) << ","
                  << "\"mean_det_ms\":" << (frames_measured ? det_ms_sum / frames_measured : 0.0) << ","
                  << "\"mean_cls_ms\":" << (frames_measured ? cls_ms_sum / frames_measured : 0.0)
                  << "}" << std::endl;
      }
    }

    double measured_sec = std::chrono::duration<double>(Clock::now() - measure_start).count();
    std::cout << "{"
              << "\"summary\":true,"
              << "\"frames\":" << frames_measured << ","
              << "\"wall_sec\":" << measured_sec << ","
              << "\"wall_fps\":" << (measured_sec > 0.0 ? frames_measured / measured_sec : 0.0) << ","
              << "\"classification_mode\":\"" << mode << "\","
              << "\"mean_pre_ms\":" << (frames_measured ? pre_ms_sum / frames_measured : 0.0) << ","
              << "\"mean_det_ms\":" << (frames_measured ? det_ms_sum / frames_measured : 0.0) << ","
              << "\"mean_cls_ms\":" << (frames_measured ? cls_ms_sum / frames_measured : 0.0)
              << "}" << std::endl;

    cudaStreamDestroy(det_stream);
    cudaStreamDestroy(cls_stream);
    cudaFreeHost(det_input);
    cudaFreeHost(image_input);
    cudaFreeHost(digit_input);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "fatal: " << e.what() << "\n";
    return 1;
  }
}
