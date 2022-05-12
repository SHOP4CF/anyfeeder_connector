/*
 * Copyright (C) 2020 Teknologisk Institut - All Rights Reserved
 */

#include "anyfeeder_connector/anyfeeder.hpp"
#include <anyfeeder_connector/connection.hpp>

#include <string>
#include <memory>
#include <map>
#include <vector>

using StandardInputSrv = anyfeeder_interfaces::srv::StandardInput;
using RestartSrv = anyfeeder_interfaces::srv::Restart;
using std::placeholders::_1;
using std::placeholders::_2;


struct Service {
    std::string name;
    void (AnyFeeder::*serviceCallback)(std::shared_ptr<StandardInputSrv::Request>,
                                       std::shared_ptr<StandardInputSrv::Response>);
    rclcpp::Service<StandardInputSrv>::SharedPtr& servicePtr;
};

AnyFeeder::AnyFeeder(std::shared_ptr<Connection>& connection) : Node("anyfeeder_node"),
                                                                connection(connection),
                                                                status(AnyFeederStatus::UnInitialized) {
    this->declare_parameter(this->PARAM_SETTLING_TIME, 0.001);
    this->declare_parameter(this->PARAM_DEVICE, "/dev/ttyUSB0");
    this->set_on_parameters_set_callback(std::bind(&AnyFeeder::parameterChanged, this, _1));
    loadDefaults();
    std::string port = this->get_parameter(this->PARAM_DEVICE).as_string();
    connection->openConnection(port);

    rclcpp::Service<StandardInputSrv>::SharedPtr ptr;

    RCLCPP_INFO(this->get_logger(), "Init services");
    Service services[] = {
            {"/init", &AnyFeeder::initialize, initService},
            {"/feed_forward", &AnyFeeder::feedForward, forwardService},
            {"/feed_backward", &AnyFeeder::feedBackward, backService},
            {"/flip_forward", &AnyFeeder::feedFlipForward, flipForwardService},
            {"/flip_backward", &AnyFeeder::feedFlipBackward, flipBackwardService},
            {"/flip", &AnyFeeder::flip, flipService},
            {"/dispense", &AnyFeeder::dispense, dispenseService},
            {"/purge", &AnyFeeder::purge, purgeService}
    };
    std::stringstream ss;
    for (Service& service : services) {
        ss.str("");
        ss << this->get_name() << service.name;
        service.servicePtr = this->create_service<StandardInputSrv>(ss.str(),
                std::bind(service.serviceCallback, this, _1, _2));
    }
    ss.str("");
    ss << this->get_name() << "/reset";
    resetErrorService = this->create_service<std_srvs::srv::Empty>(ss.str(),
            std::bind(&AnyFeeder::resetError, this, _1, _2));
    ss.str("");
    ss << this->get_name() << "/restart";
    restartService = this->create_service<RestartSrv>(ss.str(),
            std::bind(&AnyFeeder::restart, this, _1, _2));
    ss.str("");
    ss << this->get_name() << "/list_values";
    listValuesService = this->create_service<std_srvs::srv::Empty>(ss.str(),
            std::bind(&AnyFeeder::listValues, this, _1, _2));
    ss.str("");
    ss << this->get_name() << "/status";
    statusService = this->create_service<anyfeeder_interfaces::srv::Status>(ss.str(),
            std::bind(&AnyFeeder::current_status, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "I'm now initialized!");
}

AnyFeeder::~AnyFeeder() {
    connection->closeConnection();

    RCLCPP_INFO(this->get_logger(), "Killing services");
    initService.reset();
    forwardService.reset();
    backService.reset();
    flipForwardService.reset();
    flipBackwardService.reset();
    flipService.reset();
    dispenseService.reset();
    resetErrorService.reset();
    restartService.reset();
    listValuesService.reset();
    statusService.reset();
    RCLCPP_INFO(this->get_logger(), "Shutting down!");
}

void AnyFeeder::initialize(const std::shared_ptr<StandardInputSrv::Request> request,
                           std::shared_ptr<StandardInputSrv::Response> response) {
    (void) request;
    switch (status) {
        case AnyFeederStatus::Ready:
        case AnyFeederStatus::UnInitialized: {
            RCLCPP_INFO(this->get_logger(), "Executing Initialize");
            executeCommand(ABCommands::Initialize);
            RCLCPP_INFO(this->get_logger(), "AnyFeeder is done with executing the Initialize");

            AnyfeederStandardResponse result = checkForStandardResponse();
            response->status = result.status;
            response->status_msg = result.status_msg;

            if (result.status == 0) {
                response->status_msg = "Initialized";
                status = AnyFeederStatus::Ready;
            }
            break;
        }
        case AnyFeederStatus::Running:
            response->status = response->SYSTEM_RUNNING;
            response->status_msg = "System is running, initialize not allowed.";
            break;
        case AnyFeederStatus::Motor1ServoError:
            response->status = response->ERROR;
            response->status_msg = "Motor 1 Servo Error";
            break;
        case AnyFeederStatus::Motor1Error:
            response->status = response->ERROR;
            response->status_msg = "Motor 1 Error";
            break;
        case AnyFeederStatus::Motor2ServoError:
            response->status = response->ERROR;
            response->status_msg = "Motor 2 Servo Error";
            break;
        case AnyFeederStatus::Motor2Error:
            response->status = response->ERROR;
            response->status_msg = "Motor 2 Error";
            break;
        default:
            response->status = response->ERROR;
            response->status_msg = "Unknown state";
    }
}

void AnyFeeder::resetError(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                           std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    (void) request;
    (void) response;
    executeCommand(ABCommands::ResetError);
    if (checkForStandardResponse().status == 0) {
        status = AnyFeederStatus::Ready;
        RCLCPP_INFO(this->get_logger(), "Error status has been reset");
    }
}

void AnyFeeder::restart(const std::shared_ptr<RestartSrv::Request> request,
                        std::shared_ptr<RestartSrv::Response> response) {
    (void) response;
    if (request->reset.reset) {
        executeCommand(ABCommands::RestartFirmware);
        loadDefaults();
        RCLCPP_INFO(this->get_logger(), "Firmware is reset and default values are loaded");
        checkForStandardResponse();
    } else {
        executeCommand(ABCommands::StartupFirmware);
        RCLCPP_INFO(this->get_logger(), "Firmware is reset and values are kept");
    }
    status = AnyFeederStatus::UnInitialized;
}

void AnyFeeder::listValues(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                           std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    (void) request;
    (void) response;
    RCLCPP_INFO_STREAM(this->get_logger(),
            "Feed Forward Repetitions: " << getRepetitionsParameter(ABCommands::FeedForward));
    RCLCPP_INFO_STREAM(this->get_logger(),
            "Feed Forward Speed: " << getSpeedParameter(ABCommands::FeedForward));

    RCLCPP_INFO_STREAM(this->get_logger(),
            "Feed Backward Repetitions: " << getRepetitionsParameter(ABCommands::FeedBackward));
    RCLCPP_INFO_STREAM(this->get_logger(),
            "Feed Backward Speed: " << getSpeedParameter(ABCommands::FeedBackward));

    RCLCPP_INFO_STREAM(this->get_logger(),
            "Feed Flip Forward Repetitions: "
            << getRepetitionsParameter(ABCommands::FeedFlipForward));
    RCLCPP_INFO_STREAM(this->get_logger(),
            "Feed Flip Forward Speed: " << getSpeedParameter(ABCommands::FeedFlipForward));

    RCLCPP_INFO_STREAM(this->get_logger(),
            "Feed Flip Backward Repetitions: "
            << getRepetitionsParameter(ABCommands::FeedFlipBackward));
    RCLCPP_INFO_STREAM(this->get_logger(),
            "Feed Flip Backward Speed: " << getSpeedParameter(ABCommands::FeedFlipBackward));

    RCLCPP_INFO_STREAM(this->get_logger(),
            "Flip Repetitions: " << getRepetitionsParameter(ABCommands::Flip));
    RCLCPP_INFO_STREAM(this->get_logger(),
            "Flip Speed: " << getSpeedParameter(ABCommands::Flip));

    RCLCPP_INFO_STREAM(this->get_logger(),
            "Dispense Repetitions: " << getRepetitionsParameter(ABCommands::Dispense));
    RCLCPP_INFO_STREAM(this->get_logger(),
            "Dispense Speed: " << getSpeedParameter(ABCommands::Dispense));

    RCLCPP_INFO_STREAM(this->get_logger(),
            "Purge Repetitions: " << getRepetitionsParameter(ABCommands::Purge));
    RCLCPP_INFO_STREAM(this->get_logger(),
            "Purge Speed: " << getSpeedParameter(ABCommands::Purge));
}

void AnyFeeder::current_status(const std::shared_ptr<anyfeeder_interfaces::srv::Status::Request> request,
                               std::shared_ptr<anyfeeder_interfaces::srv::Status::Response> response) {
    (void) request;
    switch (status) {
        case AnyFeederStatus::UnInitialized:
            response->status = response->UN_INITIALIZED;
            break;
        case AnyFeederStatus::Ready:
            response->status = response->READY;
            break;
        case AnyFeederStatus::Running:
            response->status = response->RUNNING;
            break;
        case AnyFeederStatus::Motor1ServoError:
            response->status = response->MOTOR_1_SERVO_ERROR;
            break;
        case AnyFeederStatus::Motor1Error:
            response->status = response->MOTOR_1_ERROR;
            break;
        case AnyFeederStatus::Motor2ServoError:
            response->status = response->MOTOR_2_SERVO_ERROR;
            break;
        case AnyFeederStatus::Motor2Error:
            response->status = response->MOTOR_2_ERROR;
            break;
        case AnyFeederStatus::NoSync:
            response->status = response->NO_SYNC;
            break;
    }
    std::string status_str = "Status is: " + response->status;
    RCLCPP_WARN(this->get_logger(), status_str);
}

rcl_interfaces::msg::SetParametersResult AnyFeeder::parameterChanged(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
        if (parameter.get_name() == PARAM_SETTLING_TIME) {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                std::string message = "Invalid type: " + parameter.get_type_name() + ". Expected: double";
                result.successful = false;
                result.reason = message;
                RCLCPP_WARN(this->get_logger(), message);
                return result;
            }
        }
    }

    return result;
}

void AnyFeeder::standardInputHandler(std::shared_ptr<StandardInputSrv::Request> &request,
                                     std::shared_ptr<StandardInputSrv::Response> &response,
                                     ABCommands::com command) {
    std::string command_name;
    switch (command) {
        case ABCommands::FeedForward:
            command_name = "FeedForward";
            break;
        case ABCommands::FeedBackward:
            command_name = "FeedBackward";
            break;
        case ABCommands::FeedFlipForward:
            command_name = "FeedFlipForward";
            break;
        case ABCommands::FeedFlipBackward:
            command_name = "FeedFlipBackward";
            break;
        case ABCommands::Flip:
            command_name = "Flip";
            break;
        case ABCommands::Dispense:
            command_name = "Dispense";
            break;
        case ABCommands::Purge:
            command_name = "Purge";
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown command!");
            response->status = response->ERROR;
            response->status_msg = "Unknown command. Program error";
            return;
    }
    const int speed = request->parameters.speed;
    const int repetitions = request->parameters.repetitions;

    RCLCPP_INFO(this->get_logger(), "Called service for %s.", command_name.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(),
            "Current " << command_name << " repetitions: " << getRepetitionsParameter(command) <<
            ", new " << repetitions);
    RCLCPP_INFO_STREAM(this->get_logger(),
            "Current " << command_name << " speed: " << getSpeedParameter(command) <<
            ", new: " << speed);

    if (!checkStatus()) {
        response->status = response->NOT_READY;
        response->status_msg = "Anyfeeder might not be initialized yet. Check status.";
        return;
    }

    bool updateSpeed = (speed != getSpeedParameter(command));
    bool updateRepetitions = (repetitions != getRepetitionsParameter(command));

    if (updateSpeed && notInRange(SPEED_MIN, SPEED_MAX, speed)) {
        std::string status_msg = "Supplied speed is out of bounds! "
                                 "Range is [1," + std::to_string(SPEED_MAX) + "]";
        RCLCPP_ERROR(this->get_logger(), status_msg);
        response->status = response->INVALID_INPUT;
        response->status_msg = status_msg;
        return;
    }

    int max_value = command == ABCommands::Purge ? PURGE_REPETITIONS_MAX : REPETITIONS_MAX;
    if (updateRepetitions && notInRange(REPETITIONS_MIN, max_value, repetitions)) {
        std::string status_msg = "Supplied repetitions is out of bounds! "
                                 "Range is [1," + std::to_string(max_value) + "]";
        RCLCPP_ERROR(this->get_logger(), status_msg);
        response->status = response->INVALID_INPUT;
        response->status_msg = status_msg;
        return;
    }

    if (updateSpeed) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Sending new speed to AnyFeeder");
        setSpeed(command, speed);

        AnyfeederStandardResponse result = checkForStandardResponse();
        if (result.status != 0) {
            response->status = result.status;
            response->status_msg = result.status_msg;
            return;
        }
    }

    if (updateRepetitions) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Sending new repetitions to AnyFeeder");
        setRepetitions(command, repetitions, true);
    } else {
        RCLCPP_INFO(this->get_logger(), "Executing %s.", command_name.c_str());
        executeCommand(command);
    }

    AnyfeederStandardResponse result = checkForStandardResponse();
    response->status = result.status;
    response->status_msg = result.status_msg;

    if (result.status == 0) {
        setSpeedParameter(command, speed);
        setRepetitionsParameter(command, repetitions);
        RCLCPP_DEBUG_STREAM(this->get_logger(),
                command_name << " repetitions is now: " << repetitions);
        RCLCPP_DEBUG_STREAM(this->get_logger(),
                command_name << " speed is now: " << speed);
        double period = this->get_parameter(this->PARAM_SETTLING_TIME).as_double();
        RCLCPP_INFO_STREAM(this->get_logger(), "Waiting a bit: " << period << "[s]");
        std::this_thread::sleep_for(std::chrono::duration<double>(period));
        RCLCPP_INFO_STREAM(this->get_logger(),
                this->get_name() << " has successfully executed the command: " << command_name.c_str());
    }
}

void AnyFeeder::setRepetitions(ABCommands::com command, int repetitions, bool execute) {
    sendCommand(buildABCommand(command, 0, execute, repetitions));
}

void AnyFeeder::setSpeed(ABCommands::com command, int speed) {
    sendCommand(buildABCommand(command, 1, true, speed));
}

void AnyFeeder::executeCommand(ABCommands::com command) {
    switch(command){
        case ABCommands::FeedForward:
            sendCommand("x=1");
            break;
        case ABCommands::FeedBackward:
            sendCommand("x=2");
            break;
        case ABCommands::FeedFlipForward:
            sendCommand("x=3");
            break;
        case ABCommands::FeedFlipBackward:
            sendCommand("x=4");
            break;
        case ABCommands::Flip:
            sendCommand("x=5");
            break;
        case ABCommands::Dispense:
            sendCommand("x=6");
            break;
        case ABCommands::Purge:
            sendCommand("x=7");
            break;
        case ABCommands::Initialize:
            sendCommand("x=16");
            break;
        case ABCommands::Stop:
            sendCommand("x=15");
            break;
        case ABCommands::StartupFirmware:
            sendCommand("S RUN");
            break;
        case ABCommands::ResetError:
            sendCommand("x=30");
            break;
        case ABCommands::RestartFirmware:
            sendCommand("x=31");
            break;
    }
}

void AnyFeeder::loadDefaults() {
    // Repetitions
    this->repetitions[ABCommands::FeedForward] = DEFAULT_FEED_FORWARD_REPETITIONS;
    this->repetitions[ABCommands::FeedBackward] = DEFAULT_FEED_BACKWARD_REPETITIONS;
    this->repetitions[ABCommands::FeedFlipForward] = DEFAULT_FEED_FLIP_FORWARD_REPETITIONS;
    this->repetitions[ABCommands::FeedFlipBackward] = DEFAULT_FEED_FLIP_BACKWARD_REPETITIONS;
    this->repetitions[ABCommands::Flip] = DEFAULT_FLIP_REPETITIONS;
    this->repetitions[ABCommands::Dispense] = DEFAULT_DISPENSE_REPETITIONS;
    this->repetitions[ABCommands::Purge] = DEFAULT_PURGE_REPETITIONS;

    // Speed
    this->speed[ABCommands::FeedForward] = DEFAULT_FEED_FORWARD_SPEED;
    this->speed[ABCommands::FeedBackward] = DEFAULT_FEED_BACKWARD_SPEED;
    this->speed[ABCommands::FeedFlipForward] = DEFAULT_FEED_FLIP_FORWARD_SPEED;
    this->speed[ABCommands::FeedFlipBackward] = DEFAULT_FEED_FLIP_BACKWARD_SPEED;
    this->speed[ABCommands::Flip] = DEFAULT_FLIP_SPEED;
    this->speed[ABCommands::Dispense] = DEFAULT_DISPENSE_SPEED;
    this->speed[ABCommands::Purge] = DEFAULT_PURGE_SPEED;

    status = AnyFeederStatus::UnInitialized;
}

int AnyFeeder::getSpeedParameter(ABCommands::com command) {
    std::map<ABCommands::com, int>::const_iterator pos = this->speed.find(command);
    if (pos == this->speed.end()) {
        return -1;
    } else {
        return pos->second;
    }
}

int AnyFeeder::getRepetitionsParameter(ABCommands::com command) {
    std::map<ABCommands::com, int>::const_iterator pos = this->repetitions.find(command);
    if (pos == this->repetitions.end()) {
        return -1;
    } else {
        return pos->second;
    }
}

void AnyFeeder::setSpeedParameter(ABCommands::com command, int value) {
    this->speed[command] = value;
}

void AnyFeeder::setRepetitionsParameter(ABCommands::com command, int value) {
    this->repetitions[command] = value;
}

/**
 *
 * @param command is the command type to execute (from enum)
 * @param type is the type of operation (repetitions = 0; speed = 1)
 * @param exec is whether or not the AB is the executed version of the command
 * @param value is the value to set it to
 * @return string of the command
 */
std::string AnyFeeder::buildABCommand(ABCommands::com command, int type, bool exec, int value) {
    std::ostringstream ss;
    ss << "ab[";
    int parsedCommand = command;
    if (type == 1) {
        parsedCommand += 16;  // The difference between repetition and speed command value
    }
    ss << parsedCommand << "]=" << value;
    if (exec) {
        ss << " x=" << parsedCommand;
    }
    return ss.str();
}

std::string AnyFeeder::receiveResponse() {
    return connection->receive();
}

std::vector<std::string> AnyFeeder::receiveResponseList(unsigned int numberOfLines) {
    RCLCPP_INFO(this->get_logger(), "Awaiting response from AnyFeeder");
    std::vector<std::string> responseList;
    std::string response;

    for (unsigned int i = 0; i < numberOfLines + 1; i++) {
        response = receiveResponse();
        if (response == "timeout") {
            responseList.clear();
            responseList.push_back(response);
            return responseList;
        } else {
            if (i > 0) {
                responseList.push_back(response);
            }
        }
    }
    return responseList;
}

AnyfeederStandardResponse AnyFeeder::checkForStandardResponse() {
    std::vector<std::string> responseList = receiveResponseList(4);
    AnyfeederStandardResponse responseResult = AnyfeederStandardResponse();

    for (const std::string & response : responseList) {
        if (response == "m10")
            continue;
        if (response == "m11")
            continue;
        if (response == "m20")
            continue;
        if (response == "m21")
            continue;

        setErrorStatus(response);
        checkStatus();  // Output the error status

        if (response == "timeout") {
            responseResult.status = StandardInputSrv::Response::TIMEOUT;
        } else {
            responseResult.status = StandardInputSrv::Response::ERROR;
        }
        responseResult.status_msg = response;
    }

    return responseResult;
}

void AnyFeeder::sendCommand(const std::string& command) {
    connection->send(command);
}

void AnyFeeder::setErrorStatus(const std::string& error) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Error from AnyFeeder: " << error);
    if (error == "m13")
        status = AnyFeederStatus::Motor1ServoError;
    else if (error == "m17")
        status = AnyFeederStatus::Motor1Error;
    else if (error == "m23")
        status = AnyFeederStatus::Motor2ServoError;
    else if (error == "m27")
        status = AnyFeederStatus::Motor2Error;
    else if (error == "m28")
        status = AnyFeederStatus::NoSync;
    else
        RCLCPP_WARN(this->get_logger(), "Unknown error: " + error);
}

bool AnyFeeder::checkStatus() {
    bool ready = false;
    switch (status) {
        case AnyFeederStatus::UnInitialized:
            RCLCPP_WARN(this->get_logger(), "The AnyFeeder is not initialized. Call initialize!");
            break;
        case AnyFeederStatus::Ready:
            RCLCPP_DEBUG(this->get_logger(), "The AnyFeeder is ready to execute commands");
            ready = true;
            break;
        case AnyFeederStatus::Running:
            RCLCPP_WARN(this->get_logger(), "AnyFeeder is already executing a command");
            break;
        case AnyFeederStatus::Motor1ServoError:
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Motor 1(Flip drive) is possibly overloaded. "
                                        << "Check for obstructions or hardware problems. "
                                        << "Error must be reset before operation can continue.");
            break;
        case AnyFeederStatus::Motor1Error:
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Motor 1(Flip drive) is reporting an error. "
                                        << "Error must be reset before operation can continue");
            break;
        case AnyFeederStatus::Motor2ServoError:
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Motor 2(Dispense drive) is possibly overloaded. "
                                        << "Check for obstructions or hardware problems. "
                                        << "Error must be reset before operation can continue.");
            break;
        case AnyFeederStatus::Motor2Error:
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Motor 2(Dispense drive) is reporting an error. "
                                        << "Error must be reset before operation can continue");
            break;
        case AnyFeederStatus::NoSync:
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown error type!");
    }
    return ready;
}

bool AnyFeeder::notInRange(int min, int max, int value) {
    return value < min || value > max;
}
