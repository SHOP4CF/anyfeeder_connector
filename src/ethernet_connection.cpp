
#include "anyfeeder_connector/ethernet_connection.hpp"

#include <utility>

EthernetConnection::EthernetConnection(const rclcpp::Logger &log) : Connection(),
                                                                    log(log),
                                                                    ipAddress("172.31.1.199"),
                                                                    ipPort(4001) {

}

void EthernetConnection::openConnection(std::string port) {
    RCLCPP_INFO(log, "[openConnection] Port: " + ipAddress + ":" + std::to_string(ipPort));

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    {
        RCLCPP_ERROR(log, "Socket creation error");
        return;
    } 
   
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(ipPort); 
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, ipAddress.c_str(), &serv_addr.sin_addr)<=0)  
    {
        RCLCPP_ERROR(log, "Invalid address/Address not supported: %s", ipAddress.c_str());
        return;
    } 
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    {
        RCLCPP_ERROR(log, "Connection Failed");
    } else {
        RCLCPP_INFO(log, "Connection established to: %s", ipAddress.c_str());
    }

}

void EthernetConnection::closeConnection() {
    RCLCPP_INFO(log, "[closeConnection] ...");
    close(sock);
}

std::string EthernetConnection::receive() {
    std::string res;
    std::string return_val;
    do {
        res += return_val;
        char buffer[1] = {0};
        valread = read( sock , buffer, 1);
        return_val = buffer;
    } while (return_val != "\r");

    RCLCPP_DEBUG(log, "[receive] Result: %s", res.c_str());
    return res;
}

void EthernetConnection::send(std::string command) {
    RCLCPP_DEBUG(log, "[send] Command: " + command);
    command = command + '\r';
    ::send(sock , command.c_str() , command.length() , 0 );
}



