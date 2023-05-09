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
#include <mutex>
#include <eigen3/Eigen/Eigen>
//include <Eigen/Dense>



//#include <AdmittanceControl.h>

//AdmittanceSubscriberClass - klasa za pokretanje nodea za impedanciju/admitanciju

class AdmittanceSubscriberClass
{
public:

    bool pose_received;
    bool force_received;

    AdmittanceSubscriberClass()
    {
        // Create a node handle
        nh_ = ros::NodeHandle();

        // Subscribe/publish to the topicmod_trajectory_pub_
        force_sub_ = nh_.subscribe("red/ft_sensor", 1, &AdmittanceSubscriberClass::forceCallback, this);
        pose_sub_ = nh_.subscribe("/red/mavros/global_position/local", 1, &AdmittanceSubscriberClass::poseCallback, this);
        odom_sub_ = nh_.subscribe("red/odometry", 1, &AdmittanceSubscriberClass::odomCallback, this);
        mod_trajectory_pub_= nh_.advertise<geometry_msgs::PoseStamped>("/red/tracker/input_pose", 10);
        //mod_trajectory_pub_ = nh_.advertise<nav_msgs::Odometry>("/red/modified_trajectory", 10); 
        force_filter_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/red/filtered_force", 10);
        xc_yc_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/red/xc_yc", 10);

        HISTORY_BUFFER_SIZE = 40;

        pose_received = false;
        force_received = false;

        Tws_ << 0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        Tws_inv_ = Tws_.inverse();

        
    }


    // Callback functions
    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {
        //save data from the topic
        //ROS_INFO("Received force message");
        //mozda treba dodati i za header
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

        //geometry_msgs::WrenchStamped filtered_msg;
        filtered_msg.wrench.force.x = filtered_force_x;
        filtered_msg.wrench.force.y = filtered_force_y;
        filtered_msg.wrench.force.z = filtered_force_z;
        filtered_msg.wrench.torque.x = filtered_torque_x;
        filtered_msg.wrench.torque.y = filtered_torque_y;
        filtered_msg.wrench.torque.z = filtered_torque_z;
        force_filter_pub_.publish(filtered_msg);
        force_received = true;

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
    
    //poseCallback -> doing the god's work
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Save data from the received message
        poseCbMsg = *msg;
        pose_received = true;

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        //ROS_INFO("Received odometry message: x=%f, y=%f, z=%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        //ROS_INFO("Received odometry message");
    }

    void updatePose(const geometry_msgs::PoseStamped& pose) {
        std::lock_guard<std::mutex> lock(output_msg_mutex);
        m_output_msg = pose;
    }

    void publish() {
        std::lock_guard<std::mutex> lock(output_msg_mutex);
        mod_trajectory_pub_.publish(m_output_msg);
    }

    void run()
    {
        ROS_INFO("running");
        geometry_msgs::PoseStamped output_msg;
        geometry_msgs::WrenchStamped xc_yc_msg;

        //nav_msgs::Odometry output_msg;

        //double time_from_start = poseCbMsg.time_from_start.toSec();
        double x = poseCbMsg.pose.pose.position.x;
        double y = poseCbMsg.pose.pose.position.y;
        double z = poseCbMsg.pose.pose.position.z;

        // double vx = poseCbMsg.velocities[0].linear.x;
        // double vy = poseCbMsg.velocities[0].linear.y;
        // double vz = poseCbMsg.velocities[0].linear.z;
        
        // double ax = poseCbMsg.accelerations[0].linear.x;
        // double ay = poseCbMsg.accelerations[0].linear.y;
        // double az = poseCbMsg.accelerations[0].linear.z;

        //just casting
        double xc = filtered_msg.wrench.force.x/K;
        double yc = filtered_msg.wrench.force.y/K;
        double zc = filtered_msg.wrench.force.z/K;

        //FORCE TRANSFORM WORLD->SENSOR FRAME

        //making a force vector
        Eigen::Vector4d force_vector;
        force_vector << xc, yc, zc, 0;
        //multiplying the force vector with invers of a transformational matrix
        Eigen::Vector4d force_vector_transform = Tws_inv_* force_vector;
        //transforming the vector back to geometry_msgs::WrenchStamped
        double xct = force_vector_transform(0);
        double yct = force_vector_transform(1);  
        double zct = force_vector_transform(2);      

        //test publisher publishing
        xc_yc_msg.wrench.force.x = xct;
        xc_yc_msg.wrench.force.y = yct;
        xc_yc_msg.wrench.force.z = zct;
        xc_yc_pub_.publish(xc_yc_msg);

        // output_msg.time_from_start = ros::Duration(time_from_start);
        // output_msg.transforms.resize(1);
        // output_msg.velocities.resize(1);

        if(filtered_msg.wrench.force.x > 5 || filtered_msg.wrench.force.x < -5 || filtered_msg.wrench.force.y > 5 || filtered_msg.wrench.force.y < -5)
        {
        ROS_INFO("----- ocitala se sila veca od 5N u smjeru x/y -----");

        // Modifying the data before publishing
        output_msg.pose.position.x = xct + x;  
        output_msg.pose.position.y = yct + y;
        output_msg.pose.position.z = zct + z;
        updatePose(output_msg);

        // output_msg.pose.pose.position.x = xc + x;  
        // output_msg.pose.pose.position.y = yc + y;
        // output_msg.pose.pose.position.z = zc + z;

        //mod_trajectory_pub_.publish(output_msg); //publishing modified message
        publish();
        }

        else
        {
        // output_msg.pose.pose.position.x = x;
        // output_msg.pose.pose.position.y = y;
        // output_msg.pose.pose.position.z = z;

        output_msg.pose.position.x = x;
        output_msg.pose.position.y = y;
        output_msg.pose.position.z = z;
        }
        
        //mod_trajectory_pub_.publish(output_msg); //publishing modified message
        pose_received = false;
        force_received = false;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber force_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher mod_trajectory_pub_;
    ros::Publisher force_filter_pub_;
    ros::Publisher xc_yc_pub_;

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

    geometry_msgs::WrenchStamped filtered_msg; //Declare filtered force acting upon body
    nav_msgs::Odometry poseCbMsg; //Declare msg received from poseCb

    double K = 30; //stiffness coefficient
    double M = 40; //inertia coefficient
    double D = 40; //damping coefficient

    geometry_msgs::PoseStamped m_output_msg;
    mutable std::mutex output_msg_mutex;

    Eigen::Matrix4d Tws_;
    Eigen::Matrix4d Tws_inv_;

};

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "admittance_node");

    // Create an instance of the subscriber/publisher class
    AdmittanceSubscriberClass my_subscriber;

    ros::Rate rate(10);

    while(ros::ok()){
        if(my_subscriber.pose_received && my_subscriber.force_received){
        my_subscriber.run();
        }
        ros::spinOnce();
        //rate.sleep();
    }

    ros::shutdown(); 

    // Spin the node
    //ros::spin();

    return 0;
}
