
#include "anyfeeder_connector/mock_connection.hpp"

#include <utility>

MockConnection::MockConnection(const rclcpp::Logger &log) : Connection(),
                                                            log(log),
                                                            outputStandardResponse(false) {

}

void MockConnection::openConnection(std::string port) {
    RCLCPP_INFO(log, "[openConnection] Port: " + port);
}

void MockConnection::closeConnection() {
    RCLCPP_INFO(log, "[closeConnection] ...");
}

std::string MockConnection::receive() {
    std::string res;
    if (!responses.empty()) {
        res = responses.front();
        responses.pop();
    }
    RCLCPP_INFO(log, "[receive] Result: %s", res.c_str());
    return res;
}

void MockConnection::send(std::string command) {
    RCLCPP_INFO(log, "[send] Command: " + command);
    lastCommand = command;

    if (outputStandardResponse) {
        setReceive(std::queue<std::string>({command, "m10", "m11", "m20", "m21"}));
    }
}

void MockConnection::setReceive(std::queue<std::string> responses) {
    this->responses = std::move(responses);
}

std::string MockConnection::lastSend() {
    return lastCommand;
}

void MockConnection::setOutputStandardResponse(bool doIt) {
    outputStandardResponse = doIt;
}



