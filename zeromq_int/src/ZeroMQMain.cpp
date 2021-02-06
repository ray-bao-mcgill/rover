#include <ros/ros.h>
#include <zmq.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zeromq_int");
    ros::NodeHandle handle;

    ROS_INFO("ZeroMQ Initialized");

    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);

    std::string TOPIC = "";
    subscriber.setsockopt(ZMQ_SUBSCRIBE, TOPIC.c_str(), TOPIC.length()); // allow all messages

    int linger = 0; // Proper shutdown ZeroMQ
    subscriber.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

    subscriber.connect("tcp://localhost:6666");

    int count = 0;
    while (ros::ok())
    {
        zmq::message_t message;
        int rc = subscriber.recv(&message);
        if (rc)
        {
            count++;
            ROS_INFO("Received %d", count);
        }
    }

    // Clean up your socket and context here
    subscriber.close();
    context.close();

    return 0;
}