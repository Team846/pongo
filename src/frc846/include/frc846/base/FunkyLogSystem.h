#pragma once

#include <fmt/core.h>
#include <frc/DSControlWord.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <networktables/NetworkTableInstance.h>
#include <units/base.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <mutex>
#include <queue>
#include <type_traits>
#include <variant>

#include "frc846/base/fserver.h"

namespace frc846::base {

struct LogMessage {
  int type;                 // 0 for log, 1 for warning, 2 for error
  std::string sender;       // loggable name
  std::string content;      // formatted message
  double timestamp;         // system timestamp of the message
  int period;               // 0 for disabled, 1 for teleop, 2 for autonomous
  double period_timestamp;  // game timestamp of the message
  int char_count;           // for both sender and content

  /*
  Returns a string representation of the LogMessage struct. Use for file writes,
  logging to DS. Does NOT include newline character.
  @return: string representation of the LogMessage struct
  */
  std::string pack() {
    return std::to_string(type) + ";" + sender + ";" + content + ";" +
           std::to_string(timestamp) + ";" + std::to_string(period) + ";" +
           std::to_string(period_timestamp);
  }
};

class FunkyLogSystem {
 public:
  /*
  Spawns a new logger thread
  @param rateLimit: maximum chars/second
  */
  static void Start(int rateLimit);

  /*
  Call in main loop to update the logger with GameState (e.g. teleop, auto,
  disabled). The GameState is used in outgoing log messages.
  @param newGameState: 0 for disabled, 1 for teleop, 2 for auto
  */
  static void SetGameState(int newGameState) { gameState = newGameState; }

  static int getPeriod() { return gameState; };

  /*
  Adds a message to the sending queue. Messages will not be sent out unless
  logger has been started. Blocking operation while sending thread is reading
  from message queue.
  @param msg: LogMessage struct
  */
  static void AddMessage(LogMessage msg) {
    mtx.lock();
    messages.push(msg);
    mtx.unlock();
  }

 private:
  static void LogThread(int rateLimit, std::string logFileName);

  static int gameState;
  static std::queue<LogMessage> messages;

  static std::mutex mtx;

  static LoggingServer server;
};

class FunkyLogger {
 private:
  std::string pname_;

  float format_dp(float num, int num_places = 2) const {
    float value = (int)(num * std::pow(10, num_places) + 0.5);
    return ((float)value) / std::pow(10, num_places);
  }

  /*
  Handles a log message. Formats the message, applies custom compression, and
  sends to logging server.
  */
  template <typename... T>
  void HandleLogMessage(int type, fmt::format_string<T...> fmt,
                        T&&... args) const {
    LogMessage msg;

    msg.type = type;
    msg.sender = pname_;

    std::string temp_content = fmt::format(
        std::forward<fmt::format_string<T...>>(fmt), std::forward<T>(args)...);
    temp_content.erase(
        std::remove(temp_content.begin(), temp_content.end(), ';'),
        temp_content.end());

    msg.content = temp_content;

    msg.char_count = msg.sender.size() + msg.content.size();
    msg.timestamp = format_dp(frc::Timer::GetFPGATimestamp().to<double>(), 1);

    msg.period = FunkyLogSystem::getPeriod();
    msg.period_timestamp =
        format_dp(frc::Timer::GetMatchTime().to<double>(), 1);

    FunkyLogSystem::AddMessage(msg);
  }

 public:
  FunkyLogger(std::string pname) : pname_{pname} {};

  template <typename... T>
  void Log(fmt::format_string<T...> fmt, T&&... args) const {
    HandleLogMessage(0, fmt, std::forward<T>(args)...);
  }

  template <typename... T>
  void Warn(fmt::format_string<T...> fmt, T&&... args) const {
    HandleLogMessage(1, fmt, std::forward<T>(args)...);
  }

  template <typename... T>
  void Error(fmt::format_string<T...> fmt, T&&... args) const {
    HandleLogMessage(2, fmt, std::forward<T>(args)...);
  }
};

}  // namespace frc846::base
