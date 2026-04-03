#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#include <string>
#include <thread>
#include <chrono>
#include <iostream>

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

static void publish(ChannelPublisher<unitree_arm::msg::dds_::ArmString_>& pub,
                    unitree_arm::msg::dds_::ArmString_& msg,
                    const std::string& json)
{
    msg.data_() = json;
    pub.Write(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

int main()
{

    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    // Wait for DDS to establish
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    unitree_arm::msg::dds_::ArmString_ msg{};

    std::cout << "Starting gripper open/close loop..." << std::endl;

    int seq = 1;
    while (true)
    {
        // Open gripper (65mm) — delay_ms:2000 required for gripper to execute
        std::string open_cmd = "{\"seq\":" + std::to_string(seq++) +
            ",\"address\":1,\"funcode\":1,\"data\":{\"id\":6,\"angle\":65,\"delay_ms\":2000}}";
        publish(publisher, msg, open_cmd);
        std::cout << "Gripper OPEN (65mm)" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Close gripper (0mm)
        std::string close_cmd = "{\"seq\":" + std::to_string(seq++) +
            ",\"address\":1,\"funcode\":1,\"data\":{\"id\":6,\"angle\":0,\"delay_ms\":2000}}";
        publish(publisher, msg, close_cmd);
        std::cout << "Gripper CLOSE (0mm)" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    return 0;
}