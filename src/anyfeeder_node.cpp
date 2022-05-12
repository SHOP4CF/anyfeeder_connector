/*
 * Copyright (C) 2020 Teknologisk Institut - All Rights Reserved
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "anyfeeder_connector/anyfeeder.hpp"
#include "anyfeeder_connector/serial_connection.hpp"
#include "anyfeeder_connector/ethernet_connection.hpp"


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<Connection> serial = std::make_shared<EthernetConnection>(rclcpp::get_logger("EthernetConnection"));
    std::shared_ptr<AnyFeeder> node = std::make_shared<AnyFeeder>(serial);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}