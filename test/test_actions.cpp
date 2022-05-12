#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "anyfeeder_connector/anyfeeder.hpp"
#include "anyfeeder_connector/mock_connection.hpp"
#include "anyfeeder_interfaces/srv/standard_input.hpp"
#include "anyfeeder_interfaces/srv/status.hpp"


class AnyFeederActionTest : public ::testing::Test {
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

        serial->setOutputStandardResponse(true);
        node->initialize(request, response);
    }

    virtual void TearDown() {

    }
};

TEST_F(AnyFeederActionTest, TestFeedforward) {
    node->feedForward(request, response);
    ASSERT_STREQ("x=1", serial->lastSend().c_str());
    ASSERT_EQ(response->OK, response->status);
    ASSERT_STREQ("", response->status_msg.c_str());
}

TEST_F(AnyFeederActionTest, TestFeedforwardTooLowSpeed) {
    request->parameters.speed = 0;
    node->feedForward(request, response);

    // Initialize should be the last send command
    ASSERT_STREQ("x=16", serial->lastSend().c_str());
    ASSERT_EQ(response->INVALID_INPUT, response->status);
    ASSERT_STREQ("Supplied speed is out of bounds! Range is [1,10]", response->status_msg.c_str());
}

TEST_F(AnyFeederActionTest, TestFeedforwardTooHighSpeed) {
    request->parameters.speed = 11;
    node->feedForward(request, response);

    // Initialize should be the last send command
    ASSERT_STREQ("x=16", serial->lastSend().c_str());
    ASSERT_EQ(response->INVALID_INPUT, response->status);
    ASSERT_STREQ("Supplied speed is out of bounds! Range is [1,10]", response->status_msg.c_str());
}

TEST_F(AnyFeederActionTest, TestFeedforwardTooFewRepetitions) {
    request->parameters.repetitions = 0;
    node->feedForward(request, response);

    // Initialize should be the last send command
    ASSERT_STREQ("x=16", serial->lastSend().c_str());
    ASSERT_EQ(response->INVALID_INPUT, response->status);
    ASSERT_STREQ("Supplied repetitions is out of bounds! Range is [1,10]", response->status_msg.c_str());
}

TEST_F(AnyFeederActionTest, TestFeedforwardTooManyRepetitions) {
    request->parameters.repetitions = 11;
    node->feedForward(request, response);

    // Initialize should be the last send command
    ASSERT_STREQ("x=16", serial->lastSend().c_str());
    ASSERT_EQ(response->INVALID_INPUT, response->status);
    ASSERT_STREQ("Supplied repetitions is out of bounds! Range is [1,10]", response->status_msg.c_str());
}

TEST_F(AnyFeederActionTest, TestFeedforwardTimeout) {
    serial->setOutputStandardResponse(false);
    serial->setReceive(std::queue<std::string>({"x=1", "m11", "timeout"}));

    node->feedForward(request, response);
    ASSERT_STREQ("x=1", serial->lastSend().c_str());
    ASSERT_EQ(response->TIMEOUT, response->status);
    ASSERT_STREQ("timeout", response->status_msg.c_str());
}

TEST_F(AnyFeederActionTest, TestPurge) {
    request->parameters.repetitions = 127;  // Max repetitions for purge
    node->purge(request, response);
    ASSERT_STREQ("ab[7]=127 x=7", serial->lastSend().c_str());
    ASSERT_EQ(response->OK, response->status);
    ASSERT_STREQ("", response->status_msg.c_str());
}

TEST_F(AnyFeederActionTest, TestPurgeTooLowSpeed) {
    request->parameters.speed = 0;
    node->purge(request, response);

    // Initialize should be the last send command
    ASSERT_STREQ("x=16", serial->lastSend().c_str());
    ASSERT_EQ(response->INVALID_INPUT, response->status);
    ASSERT_STREQ("Supplied speed is out of bounds! Range is [1,10]", response->status_msg.c_str());
}

TEST_F(AnyFeederActionTest, TestPurgeTooHighSpeed) {
    request->parameters.speed = 11;
    node->purge(request, response);

    // Initialize should be the last send command
    ASSERT_STREQ("x=16", serial->lastSend().c_str());
    ASSERT_EQ(response->INVALID_INPUT, response->status);
    ASSERT_STREQ("Supplied speed is out of bounds! Range is [1,10]", response->status_msg.c_str());
}

TEST_F(AnyFeederActionTest, TestPurgeTooFewRepetitions) {
    request->parameters.repetitions = 0;
    node->purge(request, response);

    // Initialize should be the last send command
    ASSERT_STREQ("x=16", serial->lastSend().c_str());
    ASSERT_EQ(response->INVALID_INPUT, response->status);
    ASSERT_STREQ("Supplied repetitions is out of bounds! Range is [1,127]", response->status_msg.c_str());
}

TEST_F(AnyFeederActionTest, TestPurgeTooManyRepetitions) {
    request->parameters.repetitions = 128;
    node->purge(request, response);

    // Initialize should be the last send command
    ASSERT_STREQ("x=16", serial->lastSend().c_str());
    ASSERT_EQ(response->INVALID_INPUT, response->status);
    ASSERT_STREQ("Supplied repetitions is out of bounds! Range is [1,127]", response->status_msg.c_str());
}