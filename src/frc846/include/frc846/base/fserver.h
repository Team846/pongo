#pragma once

#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_set>
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
#include <string.h>
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
  LoggingServer() : messages{}, clients{} {
#ifdef _WIN32
    /*
    Configuring Windows Socket API.
    */

    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
      std::cerr << "WSAStartup failed: " << result << std::endl;
      exit(EXIT_FAILURE);
    }
#endif
  }

  ~LoggingServer() {
#ifdef _WIN32
    WSACleanup();
#endif
  }

  /*
  Starts a UDP logging server on the specified port. Make sure port is FRC legal
  in FMS. As of 2024, these include 5800...5810. Spawns threads for logging and
  watching clients.
  @param port port to start the server on
  */
  void Start(int port) {
    len = sizeof(cliaddr);

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    std::cout << sockfd << std::endl;
    if (sockfd < 0) return;

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
      return;

    std::thread sender([&]() {
      while (true) {
        do {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } while (messages.empty() || clients.empty());

        msg_mtx.lock();
        auto msg = messages.front();
        messages.pop();
        msg_mtx.unlock();

        cli_mtx.lock();
        for (const auto &client : clients) {
          sendto(sockfd, reinterpret_cast<const char *>(msg.data()), msg.size(),
                 0, reinterpret_cast<const struct sockaddr *>(&(client.addr)),
                 sizeof(client.addr));
        }
        cli_mtx.unlock();
      }
    });

    std::thread receiver([&]() {
      char buffer[1024];
      std::chrono::milliseconds lastPruneTime{getTime()};
      for (std::chrono::milliseconds t{lastPruneTime};; t = getTime()) {
        if (recvfrom(sockfd, buffer, sizeof(buffer), 0,
                     (struct sockaddr *)&cliaddr, &len) > 0) {
          bool exists = false;
          cli_mtx.lock();
          for (LoggingClient &client : clients) {
            if (client.addr.sin_addr.s_addr == cliaddr.sin_addr.s_addr) {
              if (client.addr.sin_port != cliaddr.sin_port) {
                client.addr.sin_port = cliaddr.sin_port;
              }
              client.lastKeepAlive = getTime();
              exists = true;
              break;
            }
          }
          if (!exists) {
            clients.push_back({cliaddr, getTime()});
          }
          cli_mtx.unlock();
        }
        if (t - lastPruneTime > std::chrono::milliseconds(500)) {
          cli_mtx.lock();
          for (size_t i = 0; i < clients.size(); i++) {
            if (getTime() - clients[i].lastKeepAlive >
                std::chrono::milliseconds(5000)) {
              clients.erase(clients.begin() + i);
            }
          }
          cli_mtx.unlock();
          lastPruneTime = t;
        }
      }
    });

    sender.detach();
    receiver.detach();
  }

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
