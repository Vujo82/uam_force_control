#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
//#include <AdmittanceControl.h>

class AdmittanceSubscriberClass
{
public:
    AdmittanceSubscriberClass()
    {
        // Create a node handle
        nh_ = ros::NodeHandle("~");

        // Subscribe to the topic
        sub_ = nh_.subscribe("red/ft_sensor", 10, &AdmittanceSubscriberClass::callback, this);
    }

    // Define the callback function
    void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {
        ROS_INFO("Received message: force.x = %f, force.y = %f, force.z = %f, torque.x = %f, torque.y = %f, torque.z = %f", msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);

    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "admittance_node");

    // Create an instance of the subscriber class
    AdmittanceSubscriberClass my_subscriber;

    // Spin the node
    ros::spin();

    return 0;
}
