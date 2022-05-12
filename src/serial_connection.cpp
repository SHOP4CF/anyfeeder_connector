#include "anyfeeder_connector/serial_connection.hpp"

#include <sys/types.h>
#include <fcntl.h>
#include <cstdio>
#include <string>
#include <unistd.h>
#include <termios.h>

SerialConnection::SerialConnection(const rclcpp::Logger& log) : Connection(),
                                                                log(log),
                                                                baudRate(DEFAULT_BAUD_RATE),
                                                                dataBits(DEFAULT_DATA_BITS),
                                                                stopBits(DEFAULT_STOP_BITS),
                                                                parity(DEFAULT_PARITY),
                                                                flowControl(DEFAULT_FLOW_CONTROL),
                                                                receiveBuffSize(DEFAULT_RECEIVE_BUFFER_SIZE) {
    serialPortDescriptor = -1;
    devName = "NOT_SET";
}

void SerialConnection::openConnection(std::string port) {
    RCLCPP_INFO(log, "[setupPort] Port: " + port);
    devName = port;
    serialPortDescriptor = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (serialPortDescriptor < 0) {
        throw std::out_of_range("The opening of the connection port returned a descriptor < 0!");
    } else {
        tcgetattr(serialPortDescriptor, &prevTios);
        set_serial(serialPortDescriptor, baudRate);
        RCLCPP_INFO(log, "[setupPort] Connected on port: %s", port.c_str());
    }
}

void SerialConnection::closeConnection() {
    if (serialPortDescriptor > 0) {
        tcsetattr(serialPortDescriptor, TCSANOW, &prevTios);
        close(serialPortDescriptor);
        RCLCPP_INFO(log, "AnyFeeder detached from connection port: %s", devName.c_str());
    }
}

std::string SerialConnection::receive() {
    char buffer[receiveBuffSize];
    memset(&buffer, '\0', sizeof buffer);

    int tmp_timeout = 2;
    // TODO(rlh): read tmp_timeout as parameter?

    ssize_t result_size = nonblockingSerialRead(serialPortDescriptor, &buffer, '\r',
                                                        receiveBuffSize, tmp_timeout, 0);
    if (result_size == 0) {
        RCLCPP_WARN(log, "Timed out");
        return std::string("timeout");
    }
    std::string received(buffer);
    received.erase(received.size() - 1);
    RCLCPP_INFO(log, "[receive] Received: %s", received.c_str());
    return received;
}

void SerialConnection::send(std::string command) {
    int clear_count = clear_read_buffer(serialPortDescriptor, receiveBuffSize);
    if (clear_count > 0) {
        RCLCPP_WARN(log, "The read buffer contained %d responses.", clear_count);
    }
    std::string newString = command + '\r';
    serialWrite(serialPortDescriptor, &newString[0], newString.size());
}

ssize_t SerialConnection::serialWrite(int fd, const void *buffer, ssize_t txLen) {
    int i, acc = 0;
    while (acc < txLen) {
        i = write(fd, (char*)buffer+acc, txLen - acc);
        if (i <= 0) return i;
        acc += i;
    }
    return (ssize_t)acc;
}

ssize_t SerialConnection::serialRead(int fd, void *buf, char end) {
    int n = 0, spot = 0;
    char buffer = '\0';

    do {
        n = read(fd, &buffer, 1);
        sprintf(&((char*)buf)[spot], "%c", buffer);
        spot += n;
    } while (buffer != end);

    return (ssize_t)spot;
}

ssize_t SerialConnection::nonblockingSerialRead(int fd, void *buf, char end, ssize_t txLen,
                                                int timeout_time_sec, int timeout_time_usec) {
    fd_set rset;
    int ret = 0;
    struct timeval tv{};
    tv.tv_sec = timeout_time_sec;
    tv.tv_usec = timeout_time_usec;

    // always need to zero it first, then add our new descriptor
    FD_ZERO(&rset);
    FD_SET(fd, &rset);

    if ((ret = select(fd + 1, &rset, NULL, NULL, &tv)) < 0) {
        perror("select");
    } else if(ret == 0){
        return 0;
    }

    // ret must be positive
    if (FD_ISSET(fd, &rset)) {
        ssize_t res = serialRead(fd, buf, end);
        return res;
    }
    return 0;
}

int SerialConnection::clear_read_buffer(int fd, int receiveBuffSize) {
    char buff[receiveBuffSize];
    memset(&buff, '\0', sizeof buff);
    int clearCount = 0;

    while(nonblockingSerialRead(fd, &buff, '\r', receiveBuffSize, 0, 2000) > 0){
        clearCount++;
        memset(&buff, '\0', sizeof buff);
    }
    return clearCount;
}

int SerialConnection::set_serial(int fd, int baud) {
    struct termios tios;
    bzero(&tios, sizeof(tios));

    if (tcgetattr(fd, &tios))  return -1;

    tios.c_iflag =
            IGNPAR;                     /* ignore framing errors (read as 0) */
    tios.c_oflag = 0;
    tios.c_cflag = (baud
                    | CS8
                    | CREAD       /* enable receiver ? */
                    | CLOCAL      /* ignore modem control lines */
    );
    tios.c_lflag = 0;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tios))  return -1;

    return 0;
}

