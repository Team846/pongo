#include "frc846/base/FunkyLogSystem.h"

#include <fstream>
#include <future>
#include <string>
#include <thread>

#include "frc846/base/compression.h"

namespace frc846::base {

LoggingServer FunkyLogSystem::server{};

std::mutex FunkyLogSystem::mtx{};

int FunkyLogSystem::gameState = 0;
std::queue<LogMessage> FunkyLogSystem::messages{};

void FunkyLogSystem::LogThread(int rateLimit, std::string logFileName) {
  std::string logPath = "/home/lvuser/" + logFileName;
  for (;;) {
    auto start_time = std::chrono::system_clock::now();

    int runningCharCounter = 0;

    std::string logBundle{};

    mtx.lock();

    while (!FunkyLogSystem::messages.empty()) {
      LogMessage msg = FunkyLogSystem::messages.front();
      runningCharCounter += msg.char_count;
      if (runningCharCounter > rateLimit) { break; }

      logBundle += msg.pack() + "\n";
      FunkyLogSystem::messages.pop();
    }

    mtx.unlock();

    if (logBundle.size() > 1) {
      std::ofstream log_out(
          logPath, std::fstream::in | std::fstream::out | std::fstream::trunc);
      log_out << logBundle << std::endl;
      log_out.close();

      server.AddMessage(Compression::compress(logBundle));
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(500) -
        (std::chrono::system_clock::now() - start_time));
  }
}

void FunkyLogSystem::Start(int rateLimit) {
  server.Start(5808);

  std::time_t now = std::time(nullptr);
  std::tm* now_tm = std::localtime(&now);
  std::stringstream ss;
  ss << std::put_time(now_tm, "%Y-%m-%d_%H-%M-%S");
  std::string time_str = ss.str();

  std::string logFileName = "log_" + time_str + ".log846";

  std::thread logger_thread{FunkyLogSystem::LogThread, rateLimit, logFileName};
  logger_thread.detach();
}

}  // namespace frc846::base