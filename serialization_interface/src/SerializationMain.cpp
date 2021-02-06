#include <ros/ros.h>
#include "ArmMotorCommand.h"
#include "WheelSpeed.h"
#include <unordered_set>
#include "ProcessedControllerInput.h"
#include <vector>
#include <unordered_map>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/PointCloud.h>
#include "LidarData.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace ArmControl;
using namespace DriveControl;
using namespace lidar;

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

std::unordered_set<std::string> publishing_topics;
std::unordered_map<std::string, ros::Publisher> unity_publishers;

// TODO : do not call destructors on every publish
template<typename T>
void handle_unity_incoming_simple_unmanaged_msg(const std::string& topic, const T* msg)
{
    if (publishing_topics.find(topic) == publishing_topics.end())
    {
        // new
        publishing_topics.insert(topic);
        unity_publishers[topic] = handle->advertise<T>(topic.c_str(), 5);
    }
    //ROS_INFO("Published");
    unity_publishers[topic].publish<T>(*msg);
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

template<>
void handle_unity_incoming_msg<WheelSpeed>(const std::string& topic, const uint8_t* ptr)
{
    handle_unity_incoming_simple_unmanaged_msg(topic, (const WheelSpeed*)ptr);
}

template<>
void handle_unity_incoming_msg<LidarData>(const std::string& topic, const uint8_t* ptr)
{
    ROS_INFO("LidarData received and published to %s", topic.c_str());
    handle_unity_incoming_simple_unmanaged_msg(topic, (const LidarData*)ptr);
}


std::unordered_set<std::string> subscribed_topics;
std::vector<ros::Subscriber> unity_subscribers;

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

// ros::Subscriber subtest;

//             void handle_msg_test(WheelSpeed ws)
//             {
//                 ROS_INFO("Callback!!");
//             }
// subscribing
template<>
void handle_unity_incoming_msg<void>(const std::string& topic, const uint8_t* ptr)
{
    ROS_INFO("unity subscribe called %d", (int)*ptr);
    if (subscribed_topics.find(topic) != subscribed_topics.end())
    {
        return;
    }
    subscribed_topics.insert(topic);
    
    switch (*ptr)
    {

#define CUSTOM_MSG_SUB_HANDLER(T) \
    case (uint8_t)MessageTypeCode::T : \
    { \
        boost::function<void (const T&)> func = [topicCaptured = topic](const T& msg) \
        { \
            ROS_INFO("callback for unity subscription called"); \
            UNMANAGED_SEND_TO_UNITY(T, msg, topicCaptured); \
        }; \
        unity_subscribers.push_back(handle->subscribe<T>(topic.c_str(), 3, func)); \
        ROS_INFO("client subscribed %s with type %s", topic.c_str(), #T); \ 
        break; \
    }

        CUSTOM_MSG_SUB_HANDLER(ArmMotorCommand)
        CUSTOM_MSG_SUB_HANDLER(ProcessedControllerInput)
        CUSTOM_MSG_SUB_HANDLER(LidarData)
        CUSTOM_MSG_SUB_HANDLER(WheelSpeed)
        // case ((uint8_t)MessageTypeCode::WheelSpeed):
        // {
        //     subtest = handle->subscribe<WheelSpeed>("wheel_speed", 1000, handle_msg_test);
        //     break;
        // }

        //CUSTOM_MSG_SUB_HANDLER(WheelSpeed)

#undef CUSTOM_MSG_SUB_HANDLER
#undef UNMANAGED_SEND_TO_UNITY
    }
}

void(*unity_incoming_msg_handlers[512])(const std::string& topic, const uint8_t* ptr) = {
    handle_unity_incoming_msg<ArmMotorCommand>, // 0
    handle_unity_incoming_msg<LidarData>, // 1
    handle_unity_incoming_msg<ProcessedControllerInput>, //2
    handle_unity_incoming_msg<WheelSpeed>, // 3
};

ros::Publisher pointCloudPub;

void handle_depth_image_bytes(const std_msgs::UInt8MultiArray& arr)
{
    boost::posix_time::ptime posixTimePrev = ros::Time::now().toBoost();
    // ROS_INFO("Point Cloud Size: %d", arr.data.size());
    sensor_msgs::PointCloud pc{};
    pc.header.frame_id = "world";
    std::vector<geometry_msgs::Point32> points;
    points.resize(arr.data.size() / sizeof(float) / 3);
    memcpy(points.data(), arr.data.data(), arr.data.size());

    pc.points = std::move(points);
    // pc.channels.resize(pc.points.size());
    pointCloudPub.publish<sensor_msgs::PointCloud>(pc);

    boost::posix_time::ptime posixTime = ros::Time::now().toBoost();
    std::string isoTimeStr = boost::posix_time::to_iso_extended_string(posixTime);
    uint64_t pointCloudPubTime = (posixTime - posixTimePrev).total_nanoseconds();
    ROS_INFO("Point cloud published at %s in %d ns", isoTimeStr.c_str(), pointCloudPubTime);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serialization_interface");

    unity_incoming_msg_handlers[0xFF] = handle_unity_incoming_msg<void>;

    ros::NodeHandle nodeHandle;
    handle = &nodeHandle;

    ros_to_unity_pub = handle->advertise<std_msgs::UInt8MultiArray>("ros_to_unity_topic", 3);

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
        //ROS_INFO("received msg %d %d %s", (int)*head, (int)type, topicName.c_str());
        unity_incoming_msg_handlers[(size_t)type](topicName, head);
    };
    // ros::Publisher tf2Pub = handle->advertise<tf2_msgs::TFMessage>("tf2", 10);
    ros::Subscriber sub = handle->subscribe<std_msgs::UInt8MultiArray>("unity_to_ros_topic", 3, unityToRosSub);
    pointCloudPub = handle->advertise<sensor_msgs::PointCloud>("depth_camera_point_cloud", 3);

    boost::function<void (const std_msgs::UInt8MultiArray&)> depthCamBytesHandler = [](const std_msgs::UInt8MultiArray& arr)
    {
        handle_depth_image_bytes(arr);
    };
    ros::Subscriber depthImageBytesSub = handle->subscribe<std_msgs::UInt8MultiArray>("depth_camera_point_cloud_bytes", 3, depthCamBytesHandler);
    // ros::Subscriber sub2 = handle->subscribe<WheelSpeed>("wheel_speed", 1000, +[](WheelSpeed ws){
    //     ROS_INFO("Called back test");
    // });
    //ros::Publisher pub2 = handle->advertise<WheelSpeed>("wheel_speed", 1000);
    //ros::Publisher testPub = handle->advertise<ArmMotorCommand>("test_topic", 1000);

    // tf2_ros::TransformBroadcaster broadcaster{};
    

    while (ros::ok())
    {
        // WheelSpeed ws;
        // handle_unity_incoming_msg<WheelSpeed>("wheel_speed", (uint8_t*)&ws);
        // WheelSpeed ws;
        // pub2.publish<WheelSpeed>(ws);
        // ArmMotorCommand cmd;
        // cmd.MotorVel[0] = 10;
        // cmd.MotorVel[1] = 11;
        // cmd.MotorVel[2] = 12;
        // cmd.MotorVel[3] = 13;
        // cmd.MotorVel[4] = 14;
        // cmd.MotorVel[5] = 15;
        // testPub.publish<ArmMotorCommand>(cmd);
        // broadcaster.sendTransform(geometry_msgs::TransformStamped());
        // tf2_msgs::TFMessage tf2;
        // geometry_msgs::TransformStamped transform;
        // transform.child_frame_id = "map";
        // transform.header.frame_id = "world";
        // tf2.transforms.clear();
        // tf2.transforms.push_back(transform);
        // tf2Pub.publish<tf2_msgs::TFMessage>(tf2);
        ros::spinOnce();
    }

    return 0;
}
