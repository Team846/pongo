#include "frc846/base/FunkyLogSystem.h"

#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <string>
#include <thread>

#include "frc846/base/compression.h"

namespace frc846::base {

uintmax_t FunkyLogSystem::MIN_SPACE = 25 * 1024 * 1024;  // 25mb in bytes

LoggingServer FunkyLogSystem::server{};

std::mutex FunkyLogSystem::mtx{};

int FunkyLogSystem::gameState = 0;
std::queue<LogMessage> FunkyLogSystem::messages{};

void FunkyLogSystem::LogThread(int rateLimit, std::string logFileName) {
  std::filesystem::path logDir;
#ifdef _WIN32
  logDir = "C:/Users/Public/foresting";
#elif __APPLE__
  logDir = "/Users/Shared/foresting";
#else
  logDir = "/home/lvuser/foresting";
#endif

  std::filesystem::create_directories(logDir);
  std::filesystem::path logPath = logDir / logFileName;

  std::ofstream log_out(logPath, std::ios::trunc);

  if (!log_out.is_open()) {
    std::cerr << "[ERROR] Failed to open log file: " << logPath << std::endl;
  }

  try {
    auto space_info = std::filesystem::space(logDir.root_path());
    if (space_info.available < FunkyLogSystem::MIN_SPACE) {
      std::cerr << "[WARNING] Storage space low: "
                << (space_info.available / 1024 / 1024) << "MB available. "
                << "Clearing logs directory." << std::endl;
      std::vector<std::filesystem::directory_entry> log_dirs;
      for (auto& e : std::filesystem::directory_iterator(logDir)) {
        if (e.is_regular_file() && e.path() != logPath) log_dirs.push_back(e);
      }
      if (log_dirs.size() > 50) {
        std::sort(log_dirs.begin(), log_dirs.end(), [](auto& a, auto& b) {
          return std::filesystem::last_write_time(a) <
                 std::filesystem::last_write_time(b);
        });
        for (size_t i = 0; i < log_dirs.size() - 50; i++) {
          std::filesystem::remove(log_dirs[i].path());
        }
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    std::cerr << "[ERROR] Failed to check disk space: " << e.what()
              << std::endl;
  }

  for (;;) {
    auto start_time = std::chrono::system_clock::now();

    int runningCharCounter = 0;

    std::string logBundle{};

    mtx.lock();

    while (!FunkyLogSystem::messages.empty()) {
      LogMessage msg = FunkyLogSystem::messages.front();
      runningCharCounter += msg.char_count;
      if (runningCharCounter > rateLimit) { break; }

      logBundle += msg.pack() + '\n';
      FunkyLogSystem::messages.pop();
    }

    mtx.unlock();

    if (logBundle.size() > 1) {
      if (!log_out.good()) {
        log_out.close();
        log_out.clear();
        log_out.open(logPath, std::ios::app);
        if (!log_out.is_open()) {
          std::cerr << "[ERROR] Failed to reopen log file: " << logPath
                    << std::endl;
        }
      }
      log_out << logBundle << std::endl;

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