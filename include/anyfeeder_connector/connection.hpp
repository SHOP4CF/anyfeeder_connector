#pragma once

#include <string>

class Connection {
public:
    virtual ~Connection() = default;
    virtual void openConnection(std::string port) = 0;
    virtual void closeConnection() = 0;
    virtual std::string receive() = 0;
    virtual void send(std::string) = 0;
};