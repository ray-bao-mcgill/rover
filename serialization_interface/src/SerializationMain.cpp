#include <ros/ros.h>
#include "ArmMotorCommand.h"
#include "ProcessedControllerInput.h"
#include <std_msgs/UInt8MultiArray.h>

using namespace ArmControl;

ros::NodeHandle* handle;
ros::Publisher ros_to_unity_pub;

enum class MessageTypeCode : uint64_t
{
    ArmMotorCommand = 0x00,
    LidarData = 0x01,
    ProcessedControllerInput = 0x02,
    WheelSpeed = 0x03
};

constexpr uint64_t arm_motor_command = (uint64_t)MessageTypeCode::ArmMotorCommand;
constexpr uint64_t processed_controller_input = (uint64_t)MessageTypeCode::ProcessedControllerInput;

template<typename T>
void handle_unity_incoming_msg(const std::string& topic, const uint8_t* ptr)
{
    throw std::runtime_error("Not implemented");
}

template<typename T>
void handle_unity_incoming_simple_unmanaged_msg(const std::string& topic, const T* msg)
{
    ros::Publisher topicPub = handle->advertise<T>(topic.c_str(), 10);
    topicPub.publish<T>(*msg);
}

template<>
void handle_unity_incoming_msg<ArmMotorCommand>(const std::string& topic, const uint8_t* ptr)
{
    const ArmMotorCommand* msg = (const ArmMotorCommand*)ptr;

    ROS_INFO("Publishing arm motor velocity to %s", topic.c_str());
    ROS_INFO("Vel[0]: %d", msg->MotorVel[0]);
    ROS_INFO("Vel[1]: %d", msg->MotorVel[1]);
    ROS_INFO("Vel[2]: %d", msg->MotorVel[2]);
    ROS_INFO("Vel[3]: %d", msg->MotorVel[3]);
    ROS_INFO("Vel[4]: %d", msg->MotorVel[4]);
    ROS_INFO("Vel[5]: %d", msg->MotorVel[5]);
    handle_unity_incoming_simple_unmanaged_msg(topic, msg);
}

template<>
void handle_unity_incoming_msg<ProcessedControllerInput>(const std::string& topic, const uint8_t* ptr)
{
    handle_unity_incoming_simple_unmanaged_msg(topic, (const ProcessedControllerInput*)ptr);
}



void(*unity_incoming_msg_handlers[32])(const std::string& topic, const uint8_t* ptr) = {
    handle_unity_incoming_msg<ArmMotorCommand>, // 0
    nullptr, // 1
    handle_unity_incoming_msg<ProcessedControllerInput>, //2
    nullptr // 3
};

template<typename T, T multiple>
T round_up(T num)
{
    if (num % multiple == 0)
    {
        return num;
    }

    return (num / multiple + 1) * multiple;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serialization_interface");

    ros::NodeHandle nodeHandle;
    handle = &nodeHandle;

    ros_to_unity_pub = handle->advertise<std_msgs::UInt8MultiArray>("ros_to_unity_topic", 10);

    boost::function<void (const std_msgs::UInt8MultiArray&)> unityToRosSub = [&](const std_msgs::UInt8MultiArray& arr){
        const uint8_t* head = arr.data.data();
        MessageTypeCode type = static_cast<MessageTypeCode>(head[0]);
        bool isManaged = (bool)head[1];
        head += 12;
        std::string topicName = (char*)head;
        head += topicName.length() + 1;
        if (!isManaged)
        {
            head = (const uint8_t*)round_up<uint64_t, 8>((uint64_t)head);
        }
        unity_incoming_msg_handlers[(size_t)type](topicName, head);
    };

    ros::Subscriber sub = handle->subscribe<std_msgs::UInt8MultiArray>("unity_to_ros_topic", 10, unityToRosSub);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
