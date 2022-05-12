/*
 * Copyright (C) 2020 Teknologisk Institut - All Rights Reserved
 */

#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include "connection.hpp"
#include "anyfeeder_interfaces/srv/standard_input.hpp"
#include "anyfeeder_interfaces/srv/restart.hpp"
#include "anyfeeder_interfaces/srv/status.hpp"


namespace ABCommands {
    enum com {
        FeedForward = 1,
        FeedBackward,
        FeedFlipForward,
        FeedFlipBackward,
        Flip,
        Dispense,
        Purge,
        Initialize,
        Stop,
        StartupFirmware,
        ResetError,
        RestartFirmware
    };
}  // namespace ABCommands

namespace AnyFeederStatus {
    enum Status {
        UnInitialized,
        Ready,
        Running,
        Motor1ServoError,
        Motor1Error,
        Motor2ServoError,
        Motor2Error,
        NoSync
    };
}  // namespace AnyFeederStatus

struct AnyfeederStandardResponse {
    uint status;
    std::string status_msg;
};

using StandardInputSrv = anyfeeder_interfaces::srv::StandardInput;

class AnyFeeder : public rclcpp::Node {
public:
    explicit AnyFeeder(std::shared_ptr<Connection>& connection);

    ~AnyFeeder() override;

    /*
     * ROS SERVICES
     */

    void initialize(std::shared_ptr<StandardInputSrv::Request> request,
                    std::shared_ptr<StandardInputSrv::Response> response);

    void feedForward(std::shared_ptr<StandardInputSrv::Request> request,
                     std::shared_ptr<StandardInputSrv::Response> response) {
        standardInputHandler(request, response, ABCommands::FeedForward);
    }

    void feedBackward(std::shared_ptr<StandardInputSrv::Request> request,
                      std::shared_ptr<StandardInputSrv::Response> response) {
        standardInputHandler(request, response, ABCommands::FeedBackward);
    }

    void feedFlipForward(std::shared_ptr<StandardInputSrv::Request> request,
                         std::shared_ptr<StandardInputSrv::Response> response) {
        standardInputHandler(request, response, ABCommands::FeedFlipForward);
    }

    void feedFlipBackward(std::shared_ptr<StandardInputSrv::Request> request,
                          std::shared_ptr<StandardInputSrv::Response> response) {
        standardInputHandler(request, response, ABCommands::FeedFlipBackward);
    }

    void flip (std::shared_ptr<StandardInputSrv::Request> request,
               std::shared_ptr<StandardInputSrv::Response> response) {
        standardInputHandler(request, response, ABCommands::Flip);
    }

    void dispense (std::shared_ptr<StandardInputSrv::Request> request,
                   std::shared_ptr<StandardInputSrv::Response> response) {
        standardInputHandler(request, response, ABCommands::Dispense);
    }

    void purge (std::shared_ptr<StandardInputSrv::Request> request,
                std::shared_ptr<StandardInputSrv::Response> response) {
        standardInputHandler(request, response, ABCommands::Purge);
    }

    void resetError (std::shared_ptr<std_srvs::srv::Empty::Request> request,
                     std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void restart (std::shared_ptr<anyfeeder_interfaces::srv::Restart::Request> request,
                  std::shared_ptr<anyfeeder_interfaces::srv::Restart::Response> response);

    void listValues (std::shared_ptr<std_srvs::srv::Empty::Request> request,
                     std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void current_status (std::shared_ptr<anyfeeder_interfaces::srv::Status::Request> request,
                         std::shared_ptr<anyfeeder_interfaces::srv::Status::Response> response);


private:
    rcl_interfaces::msg::SetParametersResult parameterChanged(const std::vector<rclcpp::Parameter>& parameters);

    void standardInputHandler (
            std::shared_ptr<StandardInputSrv::Request> &request,
            std::shared_ptr<StandardInputSrv::Response> &response,
            ABCommands::com command);

    void setRepetitions(ABCommands::com command, int repetitions, bool execute);

    void setSpeed(ABCommands::com command, int speed);

    void executeCommand(ABCommands::com command);

    void loadDefaults();

    int getSpeedParameter(ABCommands::com command);
    int getRepetitionsParameter(ABCommands::com command);
    void setSpeedParameter(ABCommands::com command, int value);
    void setRepetitionsParameter(ABCommands::com command, int value);


    std::string buildABCommand(ABCommands::com command, int type, bool exec, int value);

    std::string receiveResponse();
    std::vector<std::string> receiveResponseList(unsigned int numberOfLines);
    AnyfeederStandardResponse checkForStandardResponse();

    void sendCommand(const std::string& command);

    void setErrorStatus(const std::string& error);

    bool checkStatus();

    bool notInRange(int min, int max, int value);

    /*
     * VARIABLES
     */

    // Services
    rclcpp::Service<StandardInputSrv>::SharedPtr initService;
    rclcpp::Service<StandardInputSrv>::SharedPtr forwardService;
    rclcpp::Service<StandardInputSrv>::SharedPtr backService;
    rclcpp::Service<StandardInputSrv>::SharedPtr flipForwardService;
    rclcpp::Service<StandardInputSrv>::SharedPtr flipBackwardService;
    rclcpp::Service<StandardInputSrv>::SharedPtr flipService;
    rclcpp::Service<StandardInputSrv>::SharedPtr dispenseService;
    rclcpp::Service<StandardInputSrv>::SharedPtr purgeService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetErrorService;
    rclcpp::Service<anyfeeder_interfaces::srv::Restart>::SharedPtr restartService;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr listValuesService;
    rclcpp::Service<anyfeeder_interfaces::srv::Status>::SharedPtr statusService;

    std::shared_ptr<Connection> connection;

    // AnyFeeder variables
    std::map<ABCommands::com, int> repetitions;
    std::map<ABCommands::com, int> speed;

    AnyFeederStatus::Status status;

    /*
     * CONSTANTS
     */
    // PARAMS
    const std::string PARAM_DEVICE = "device";
    const std::string PARAM_SETTLING_TIME = "settling_time";

    // AnyFeeder default bootup settings
    static const int DEFAULT_FEED_FORWARD_REPETITIONS = 3;
    static const int DEFAULT_FEED_BACKWARD_REPETITIONS = 3;
    static const int DEFAULT_FEED_FLIP_FORWARD_REPETITIONS = 3;
    static const int DEFAULT_FEED_FLIP_BACKWARD_REPETITIONS = 3;
    static const int DEFAULT_FLIP_REPETITIONS = 3;
    static const int DEFAULT_DISPENSE_REPETITIONS = 3;
    static const int DEFAULT_PURGE_REPETITIONS = 64;
    static const int DEFAULT_FEED_FORWARD_SPEED = 1;
    static const int DEFAULT_FEED_BACKWARD_SPEED = 1;
    static const int DEFAULT_FEED_FLIP_FORWARD_SPEED = 1;
    static const int DEFAULT_FEED_FLIP_BACKWARD_SPEED = 1;
    static const int DEFAULT_FLIP_SPEED = 1;
    static const int DEFAULT_DISPENSE_SPEED = 1;
    static const int DEFAULT_PURGE_SPEED = 1;

    // AnyFeeder limitations
    static const int SPEED_MIN = 1;
    static const int SPEED_MAX = 10;
    static const int REPETITIONS_MIN = 1;
    static const int REPETITIONS_MAX = 10;
    static const int PURGE_REPETITIONS_MAX = 127;
};
