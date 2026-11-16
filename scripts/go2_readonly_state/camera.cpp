#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void onSignal(int) { stop_requested = 1; }
}

int main(int argc, char** argv) {
  // stdout protocol: one JSON header line, followed by exactly `bytes` JPEG bytes.
  if (argc != 5 || std::string(argv[1]) != "--domain" ||
      std::string(argv[3]) != "--interface") {
    std::cerr << "Usage: " << argv[0] << " --domain <N> --interface <NIC>\n";
    return 2;
  }
  int domain;
  try {
    size_t used;
    domain = std::stoi(argv[2], &used);
    if (argv[2][used] != '\0') throw std::invalid_argument("DDS domain");
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  bool initialized = false;
  int result = 0;
  try {
    unitree::robot::ChannelFactory::Instance()->Init(domain, argv[4]);
    initialized = true;
    {
      unitree::robot::go2::VideoClient client;
      client.SetTimeout(2.0f);
      client.Init();
      std::cerr << "Go2 front camera: GetImageSample only; no motion commands\n";
      while (!stop_requested) {
        std::vector<uint8_t> jpeg;
        const int code = client.GetImageSample(jpeg);
        const size_t bytes = code == 0 ? jpeg.size() : 0;
        std::cout << "{\"code\":" << code << ",\"bytes\":" << bytes << "}\n";
        if (bytes) std::cout.write(reinterpret_cast<const char*>(jpeg.data()), bytes);
        std::cout.flush();
        if (!std::cout) break;  // GUI has closed its pipe.
        std::this_thread::sleep_for(std::chrono::milliseconds(code == 0 ? 100 : 500));
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "Camera request failed: " << error.what() << '\n';
    result = 1;
  }
  if (initialized) unitree::robot::ChannelFactory::Instance()->Release();
  std::cerr << "Camera reader exited\n";
  return result;
}
