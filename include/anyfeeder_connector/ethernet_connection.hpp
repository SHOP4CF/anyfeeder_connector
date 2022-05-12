#pragma once

#include "connection.hpp"
#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <stdio.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h> 

class EthernetConnection : public Connection {
public:
    explicit EthernetConnection(const rclcpp::Logger& log);
    void openConnection(std::string port) override;
    void closeConnection() override;
    std::string receive() override;
    void send(std::string command) override;

private:
    rclcpp::Logger log;

    std::string ipAddress;
    int ipPort;
    int sock = 0, valread;
    struct sockaddr_in serv_addr; 
    char buffer[1024] = {0};
};