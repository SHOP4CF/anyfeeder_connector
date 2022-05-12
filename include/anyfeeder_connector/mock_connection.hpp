#pragma once

#include "connection.hpp"
#include <rclcpp/rclcpp.hpp>
#include <queue>

class MockConnection : public Connection {
public:
    explicit MockConnection(const rclcpp::Logger& log);
    void openConnection(std::string port) override;
    void closeConnection() override;
    std::string receive() override;
    void send(std::string command) override;

    void setReceive(std::queue<std::string> responses);
    std::string lastSend();
    void setOutputStandardResponse(bool doIt);

private:
    rclcpp::Logger log;

    std::queue<std::string> responses;
    std::string lastCommand;
    bool outputStandardResponse;
};