#include "frc846/base/fserver.h"

#include <string.h>

#include <algorithm>
#include <thread>

namespace frc846::base {

LoggingServer::LoggingServer() : messages{}, clients{} {
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

LoggingServer::~LoggingServer() {
#ifdef _WIN32
  WSACleanup();
#endif
}

void LoggingServer::Start(int port) {
  len = sizeof(cliaddr);

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) throw std::runtime_error("Socket creation failed");

  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(port);

  if (bind(sockfd, reinterpret_cast<const struct sockaddr *>(&servaddr),
          sizeof(servaddr)) < 0)
    throw std::runtime_error("Socket bind failed");

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
              reinterpret_cast<struct sockaddr *>(&cliaddr), &len) > 0) {
        cli_mtx.lock();
        auto it = std::find_if(
            clients.begin(), clients.end(), [&](const LoggingClient &client) {
              return client.addr.sin_addr.s_addr == cliaddr.sin_addr.s_addr;
            });

        if (it != clients.end()) {
          it->addr.sin_port = cliaddr.sin_port;
          it->lastKeepAlive = getTime();
        } else {
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

}  // namespace frc846::base