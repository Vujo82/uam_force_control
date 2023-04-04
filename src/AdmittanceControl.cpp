#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
//#include <AdmittanceControl.h>

class AdmittanceSubscriberClass
{
public:
    AdmittanceSubscriberClass()
    {
        // Create a node handle
        nh_ = ros::NodeHandle();

        // Subscribe to the topic
        force_sub_ = nh_.subscribe("red/ft_sensor", 1, &AdmittanceSubscriberClass::forceCallback, this);
        pose_sub_ = nh_.subscribe("red/pose", 1, &AdmittanceSubscriberClass::poseCallback, this);
        odom_sub_ = nh_.subscribe("red/odometry", 1, &AdmittanceSubscriberClass::odomCallback, this);

    }

    // Define the callback function
    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {
        ROS_INFO("Received message: force.x = %f, force.y = %f, force.z = %f, torque.x = %f, torque.y = %f, torque.z = %f", msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_INFO("Received pose message: x=%f, y=%f, z=%f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        ROS_INFO("Received odometry message: x=%f, y=%f, z=%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber force_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
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
