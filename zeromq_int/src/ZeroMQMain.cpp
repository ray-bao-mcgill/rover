#include <ros/ros.h>
#include <zmq.hpp>
#include <sensor_msgs/PointCloud.h>

ros::NodeHandle* handle;
ros::Publisher* pc;

template<typename T>
void send_to_ros(const T& arr) {
    sensor_msgs::PointCloud cloud{};
    cloud.header.frame_id = "world";
    std::vector<geometry_msgs::Point32> points;
    points.resize(arr.size() / sizeof(float) / 3);
    memcpy(points.data(), arr.data(), arr.size());

    cloud.points = std::move(points);
    // pc.channels.resize(pc.points.size());
    pc->publish<sensor_msgs::PointCloud>(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zeromq_int", ros::init_options::AnonymousName);
    ros::NodeHandle h{};
    handle = &h;
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);

    std::string TOPIC = "";
    subscriber.setsockopt(ZMQ_SUBSCRIBE, TOPIC.c_str(), TOPIC.length()); // allow all messages

    int linger = 0; // Proper shutdown ZeroMQ
    subscriber.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

    subscriber.connect("tcp://192.168.159.1:12345");
    ros::Publisher pc = h.advertise<sensor_msgs::PointCloud>("depth_camera_point_cloud", 1);
    ::pc = &pc;
    int count = 0;
    while (ros::ok())
    {
        zmq::message_t message;
        int rc = subscriber.recv(&message);
        char* ptr = (char*)message.data();
        if (rc)
        {
            count++;
            //ROS_INFO("Received %d %d %d %d %d", count, (int)ptr[0], (int)ptr[100], (int)ptr[1000], (int)ptr[100000]);
            ROS_INFO("Received %d", count);
            std::vector<uint8_t> bytes;
            bytes.resize(message.size());
            memcpy(bytes.data(), ptr, message.size());
            send_to_ros(bytes);
        }
    }

    // Clean up your socket and context here
    subscriber.close();
    context.close();
    return 0;
}