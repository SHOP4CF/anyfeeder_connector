#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "anyfeeder_connector/anyfeeder.hpp"
#include "anyfeeder_connector/mock_connection.hpp"
#include "anyfeeder_interfaces/srv/standard_input.hpp"
#include "anyfeeder_interfaces/srv/status.hpp"


class AnyTest : public ::testing::Test {
protected:
    std::shared_ptr<MockConnection> serial;
    std::shared_ptr<AnyFeeder> node;
    std::shared_ptr<StandardInputSrv::Request> request;
    std::shared_ptr<StandardInputSrv::Response> response;
    std::shared_ptr<anyfeeder_interfaces::srv::Status::Request> status_request;
    std::shared_ptr<anyfeeder_interfaces::srv::Status::Response> status_response;

    virtual void SetUp() {
        serial = std::make_shared<MockConnection>(rclcpp::get_logger("MockConnection"));
        std::shared_ptr<Connection> s = serial;
        node = std::make_shared<AnyFeeder>(s);
        request = std::make_shared<StandardInputSrv::Request>();
        request->parameters.speed = 1;
        request->parameters.repetitions = 3;
        response = std::make_shared<StandardInputSrv::Response>();

        status_request = std::make_shared<anyfeeder_interfaces::srv::Status::Request>();
        status_response = std::make_shared<anyfeeder_interfaces::srv::Status::Response>();
    }

    virtual void TearDown() {

    }
};

TEST_F(AnyTest, TestNodeName) {
    ASSERT_STREQ("anyfeeder_node", node->get_name());
}

TEST_F(AnyTest, TestInitialStatus) {
    // Check status
    node->current_status(status_request, status_response);
    ASSERT_EQ(status_response->UN_INITIALIZED, status_response->status);
}


TEST_F(AnyTest, TestInitialize) {
    // Initialize
    serial->setOutputStandardResponse(true);
    node->initialize(request, response);
    ASSERT_STREQ("x=16", serial->lastSend().c_str());
    ASSERT_EQ(uint8_t(0), response->status);
    ASSERT_STREQ("Initialized", response->status_msg.c_str());

    // Check status
    node->current_status(status_request, status_response);
    ASSERT_EQ(status_response->READY, status_response->status);
}

TEST_F(AnyTest, TestInitializeMotor1Error) {
    serial->setReceive(std::queue<std::string>({"x=16", "m11", "m21", "m10", "m27"}));
    node->initialize(request, response);
    ASSERT_STREQ("x=16", serial->lastSend().c_str());
    ASSERT_EQ(uint8_t(1), response->status);
    ASSERT_STREQ("m27", response->status_msg.c_str());
    // TODO(rlh): this should return "Motor 2 Error" instead of "m27".

    // Check status
    node->current_status(status_request, status_response);
    ASSERT_EQ(status_response->MOTOR_2_ERROR, status_response->status);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    return RUN_ALL_TESTS();
}