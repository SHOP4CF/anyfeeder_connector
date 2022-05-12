#pragma once

#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include "connection.hpp"

class SerialConnection : public Connection {
public:
    explicit SerialConnection(const rclcpp::Logger& log);

    void openConnection(std::string port) override;
    void closeConnection() override;
    std::string receive() override;
    void send(std::string) override;

private:
    ssize_t serialWrite(int fd, const void *buffer, ssize_t txLen);
    ssize_t serialRead(int fd, void *buffer, char end);
    ssize_t nonblockingSerialRead(int fd, void *buf, char end,
                                  ssize_t txLen, int timeout_time_sec, int timeout_time_usec);
    int clear_read_buffer(int fd, int receiveBuffSize);
    int set_serial(int fd, int baud);

    rclcpp::Logger log;

    // Serial connection settings
    int serialPortDescriptor;
    int baudRate;
    char dataBits;
    char stopBits;
    char parity;
    char flowControl;
    char receiveBuffSize;

    std::string devName;
    struct termios prevTios{};

    // Serial port default settings per AnyFeeder datasheet
    static const int DEFAULT_BAUD_RATE = B9600;
    static const int DEFAULT_DATA_BITS = 8;
    static const int DEFAULT_STOP_BITS = 1;
    static const int DEFAULT_PARITY = 0;
    static const int DEFAULT_FLOW_CONTROL = 0;
    static const int DEFAULT_RECEIVE_BUFFER_SIZE = 10;
};