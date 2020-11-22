#include <ros/ros.h>
#include "ArmMotorCommand.h"
#include <unordered_set>
#include "ProcessedControllerInput.h"
#include <vector>
#include <std_msgs/UInt8MultiArray.h>

using namespace ArmControl;

ros::NodeHandle* handle;
ros::Publisher ros_to_unity_pub;

enum class MessageTypeCode : uint8_t
{
    ArmMotorCommand = 0x00,
    LidarData = 0x01,
    ProcessedControllerInput = 0x02,
    WheelSpeed = 0x03
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

    // ROS_INFO("Publishing arm motor velocity to %s", topic.c_str());
    // ROS_INFO("Vel[0]: %d", msg->MotorVel[0]);
    // ROS_INFO("Vel[1]: %d", msg->MotorVel[1]);
    // ROS_INFO("Vel[2]: %d", msg->MotorVel[2]);
    // ROS_INFO("Vel[3]: %d", msg->MotorVel[3]);
    // ROS_INFO("Vel[4]: %d", msg->MotorVel[4]);
    // ROS_INFO("Vel[5]: %d", msg->MotorVel[5]);
    handle_unity_incoming_simple_unmanaged_msg(topic, msg);
}

template<>
void handle_unity_incoming_msg<ProcessedControllerInput>(const std::string& topic, const uint8_t* ptr)
{
    handle_unity_incoming_simple_unmanaged_msg(topic, (const ProcessedControllerInput*)ptr);
}


std::unordered_set<std::string> subscribed_topics;
std::unordered_set<ros::Subscriber*> unity_subscribers;

template<typename T>
void unmanaged_send_to_unity(const T& msg, MessageTypeCode typeCode, const std::string& topic)
{
    ROS_INFO("Trying to send message of type %d from topic %s to unity", (int)typeCode, topic.c_str());
    std_msgs::UInt8MultiArray arr;
    std::vector<uint8_t>& data = arr.data;
    data.resize(8 + 4 + topic.length() + 1 + sizeof(T) + 8);
    uint8_t* head = data.data();
    head[0] = (uint8_t)typeCode;
    head[1] = 0; // unmanaged
    head += 8;
    *(int*)head = topic.length();
    head += sizeof(int);
    memcpy(head, topic.data(), topic.length());
    head += topic.length();
    *head = 0;
    head++;
    head = (uint8_t*)round_up<uint64_t, 8>((uint64_t)head); // ensure alignment
    memcpy(head, &msg, sizeof(T));
    ros_to_unity_pub.publish<std_msgs::UInt8MultiArray>(arr);
}

#define UNMANAGED_SEND_TO_UNITY(T, msg, topic) \
    { \
        MessageTypeCode typeCode = MessageTypeCode::T; \
        unmanaged_send_to_unity(msg, typeCode, topic); \
    }

template<>
void handle_unity_incoming_msg<void>(const std::string& topic, const uint8_t* ptr)
{
    ROS_INFO("unity subscribe called %d", (int)*ptr);
    if (subscribed_topics.find(topic) != subscribed_topics.end())
    {
        return;
    }
    subscribed_topics.insert(topic);
    ros::Subscriber msgSubscriber;
    
    switch (*ptr)
    {

#define CUSTOM_MSG_SUB_HANDLER(T) \
    case (uint8_t)MessageTypeCode::T : \
    { \
        boost::function<void (const T&)> func = [topicCaptured = topic](const T& msg) \
        { \
            UNMANAGED_SEND_TO_UNITY(T, msg, topicCaptured); \
        }; \
        msgSubscriber = handle->subscribe<T>(topic.c_str(), 10, func); \
        ROS_INFO("client subscribed %s with type %s", topic.c_str(), #T); \ 
        break; \
    }

        CUSTOM_MSG_SUB_HANDLER(ArmMotorCommand)
        CUSTOM_MSG_SUB_HANDLER(ProcessedControllerInput)
        case (uint8_t)MessageTypeCode::LidarData:
        {
            break;
        }

        case (uint8_t)MessageTypeCode::WheelSpeed:
        {
            break;
        }

#undef CUSTOM_MSG_SUB_HANDLER
#undef UNMANAGED_SEND_TO_UNITY
    }
    ros::Subscriber* heapSub = new ros::Subscriber;
    *heapSub = std::move(msgSubscriber);
    unity_subscribers.insert(heapSub);
}

void(*unity_incoming_msg_handlers[512])(const std::string& topic, const uint8_t* ptr) = {
    handle_unity_incoming_msg<ArmMotorCommand>, // 0
    nullptr, // 1
    handle_unity_incoming_msg<ProcessedControllerInput>, //2
    nullptr, // 3
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serialization_interface");

    unity_incoming_msg_handlers[0xFF] = handle_unity_incoming_msg<void>;

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
        ROS_INFO("received msg %d %d %s", (int)*head, (int)type, topicName.c_str());
        unity_incoming_msg_handlers[(size_t)type](topicName, head);
    };

    ros::Subscriber sub = handle->subscribe<std_msgs::UInt8MultiArray>("unity_to_ros_topic", 10, unityToRosSub);

    ros::Publisher testPub = handle->advertise<ArmMotorCommand>("test_topic", 10);
    while (ros::ok())
    {
        ArmMotorCommand cmd;
        cmd.MotorVel[0] = 10;
        cmd.MotorVel[1] = 11;
        cmd.MotorVel[2] = 12;
        cmd.MotorVel[3] = 13;
        cmd.MotorVel[4] = 14;
        cmd.MotorVel[5] = 15;
        testPub.publish<ArmMotorCommand>(cmd);
        ros::spinOnce();
    }

    return 0;
}
