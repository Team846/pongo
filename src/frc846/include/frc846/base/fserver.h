#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <queue>
#include <vector>

#ifdef _WIN32

/*
Importing windows-specific networking libraries.
Required for hardware simulation.
*/

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;

#else

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#endif

namespace frc846::base {

struct LoggingClient {
  sockaddr_in addr;
  std::chrono::milliseconds lastKeepAlive;
};

class LoggingServer {
private:
  std::chrono::milliseconds getTime() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
  }

public:
  LoggingServer();
  ~LoggingServer();

  /*
  Starts a UDP logging server on the specified port. Make sure port is FRC legal
  in FMS. As of 2024, these include 5800...5810. Spawns threads for logging and
  watching clients.
  @param port port to start the server on
  */
  void Start(int port);

  /*
  Adds message to the sending queue. Messages will not be sent out unless server
  has been started. Blocking operation while server is reading message.
  */
  void AddMessage(const std::vector<uint8_t> &message) {
    msg_mtx.lock();
    messages.push(message);
    msg_mtx.unlock();
  }

private:
  std::queue<std::vector<uint8_t>> messages;
  std::vector<LoggingClient> clients;

  std::mutex msg_mtx;
  std::mutex cli_mtx;

  int sockfd;
  sockaddr_in servaddr, cliaddr;
  socklen_t len;
};

}  // namespace frc846::base
