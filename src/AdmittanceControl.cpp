#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <deque>


//#include <AdmittanceControl.h>

//AdmittanceSubscriberClass - klasa za ocitavanje subscribera

class AdmittanceSubscriberClass
{
public:
    AdmittanceSubscriberClass()
    {
        // Create a node handle
        nh_ = ros::NodeHandle();

        // Subscribe/publish to the topic
        force_sub_ = nh_.subscribe("red/ft_sensor", 1, &AdmittanceSubscriberClass::forceCallback, this);
        pose_sub_ = nh_.subscribe("/red/position_hold/trajectory", 1, &AdmittanceSubscriberClass::poseCallback, this);
        odom_sub_ = nh_.subscribe("red/odometry", 1, &AdmittanceSubscriberClass::odomCallback, this);
        mod_trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/red/modified_trajectory", 10);
        force_filter_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/red/filtered_force", 10);

        HISTORY_BUFFER_SIZE = 50;
    }

    // Callback functions
    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {
        //save data from the topic
        ROS_INFO("Received force message");
        double force_x = msg->wrench.force.x;
        double force_y = msg->wrench.force.y;
        double force_z = msg->wrench.force.z;
        double torque_x = msg->wrench.torque.x;
        double torque_y = msg->wrench.torque.y;
        double torque_z = msg->wrench.torque.z;

        //computing the median and mean 
        double median_force_x = computeMedian(force_x, force_history_x_, HISTORY_BUFFER_SIZE);
        double median_force_y = computeMedian(force_y, force_history_y_, HISTORY_BUFFER_SIZE);
        double median_force_z = computeMedian(force_z, force_history_z_, HISTORY_BUFFER_SIZE);
        double median_torque_x = computeMedian(torque_x, torque_history_x_, HISTORY_BUFFER_SIZE);
        double median_torque_y = computeMedian(torque_y, torque_history_y_, HISTORY_BUFFER_SIZE);
        double median_torque_z = computeMedian(torque_z, torque_history_z_, HISTORY_BUFFER_SIZE);

        double filtered_force_x = computeMean(median_force_x, filt_force_history_x_, HISTORY_BUFFER_SIZE);
        double filtered_force_y = computeMean(median_force_y, filt_force_history_y_, HISTORY_BUFFER_SIZE);
        double filtered_force_z = computeMean(median_force_z, filt_force_history_z_, HISTORY_BUFFER_SIZE);
        double filtered_torque_x = computeMean(median_torque_x, filt_torque_history_x_, HISTORY_BUFFER_SIZE);
        double filtered_torque_y = computeMean(median_torque_y, filt_torque_history_y_, HISTORY_BUFFER_SIZE);
        double filtered_torque_z = computeMean(median_torque_z, filt_torque_history_z_, HISTORY_BUFFER_SIZE);

        geometry_msgs::WrenchStamped filtered_msg;
        filtered_msg.wrench.force.x = filtered_force_x;
        filtered_msg.wrench.force.y = filtered_force_y;
        filtered_msg.wrench.force.z = filtered_force_z;
        filtered_msg.wrench.torque.x = filtered_torque_x;
        filtered_msg.wrench.torque.y = filtered_torque_y;
        filtered_msg.wrench.torque.z = filtered_torque_z;
        force_filter_pub_.publish(filtered_msg);

    }

    double computeMedian(double data, std::deque<double>& history, int windowSize)
    {
        // adding data to the history
        history.push_back(data);

        // check if history size exceeds windowSize
        if (history.size() > windowSize)
        {
            history.pop_front(); // remove oldest data if history size exceeds windowSize
        }

        // get history size
        int size = history.size();

        // computing the median of the history
        std::deque<double> medianHistory = history; // create a copy of history for median filtering
        std::sort(medianHistory.begin(), medianHistory.end()); // sort the median history
        double median;
        if (size % 2 == 0)
        {
            // If the size is even, average the middle two elements
            int mid1 = size / 2 - 1;
            int mid2 = size / 2;
            median = (medianHistory[mid1] + medianHistory[mid2]) / 2.0;
        }
        else
        {
            // If the size is odd, return the middle element as the median
            median = medianHistory[size / 2];
        }

        // return the median
        return median;
    }

    double computeMean(double data, std::deque<double>& history, int windowSize)
    {
        // adding data to the history
        history.push_back(data);

        // check if history size exceeds windowSize
        if (history.size() > windowSize)
        {
            history.pop_front(); // remove oldest data if history size exceeds windowSize
        }

        // get history size
        int size = history.size();

        // computing the mean of the history
        double sum = 0;
        for (int i = 0; i < size; i++)
        {
            sum += history[i];
        }
        double mean = sum / size;

        return mean;
    }
    
    //receiving data from /red/position_hold/trajectory and posting data*2 to /red/custom_pub topic
    void poseCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg)
    {
        //ROS_INFO("Received pose message linear_velocity_x=%f", msg->velocities[0].linear.x);
        // Extract data from the received message
        double time_from_start = msg->time_from_start.toSec();
        double x = msg->transforms[0].translation.x;
        double y = msg->transforms[0].translation.y;
        double z = msg->transforms[0].translation.z;
        double vx = msg->velocities[0].linear.x;
        double vy = msg->velocities[0].linear.y;
        double vz = msg->velocities[0].linear.z;

        // Print the extracted data
        ROS_INFO("Time from start: %.2f", time_from_start);
        ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", x, y, z);
        ROS_INFO("Linear velocity: vx=%.2f, vy=%.2f, vz=%.2f", vx, vy, vz);

        // Publish the extracted data on a new topic
        trajectory_msgs::MultiDOFJointTrajectoryPoint output_msg;
        output_msg.time_from_start = ros::Duration(time_from_start);
        output_msg.transforms.resize(1);
        output_msg.velocities.resize(1);
        output_msg.transforms[0].translation.x = x * 2;  // Modifying the data before publishing
        output_msg.transforms[0].translation.y = y * 2;
        output_msg.transforms[0].translation.z = z * 2;
        output_msg.velocities[0].linear.x = vx * 2;
        output_msg.velocities[0].linear.y = vy * 2;
        output_msg.velocities[0].linear.z = vz * 2;
        mod_trajectory_pub_.publish(output_msg);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        //ROS_INFO("Received odometry message: x=%f, y=%f, z=%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        ROS_INFO("Received odometry message");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber force_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher mod_trajectory_pub_;
    ros::Publisher force_filter_pub_;

    //median history
    std::deque<double> force_history_x_; // Declare a deque for storing force values
    std::deque<double> force_history_y_;
    std::deque<double> force_history_z_;

    std::deque<double> torque_history_x_; // Declare a deque for storing torque values
    std::deque<double> torque_history_y_;
    std::deque<double> torque_history_z_;

    //median+mean history
    std::deque<double> filt_force_history_x_; // Declare a deque for storing force values
    std::deque<double> filt_force_history_y_;
    std::deque<double> filt_force_history_z_;

    std::deque<double> filt_torque_history_x_; // Declare a deque for storing torque values
    std::deque<double> filt_torque_history_y_;
    std::deque<double> filt_torque_history_z_;

    int HISTORY_BUFFER_SIZE; // Declare the size of the history buffer
};


int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "admittance_node");

    // Create an instance of the subscriber/publisher class
    AdmittanceSubscriberClass my_subscriber; 

    // Spin the node
    ros::spin();

    return 0;
}
