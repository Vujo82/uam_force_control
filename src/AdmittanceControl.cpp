#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
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
#include <dynamic_reconfigure/server.h>
#include </root/uav_ws/src/uam_force_control/cfg/cpp/uam_force_control/parametersConfig.h>
#include <numeric>
#include <cmath>
//#include "/root/uav_ws/devel/include/uam_force_control/MyBoolean.h"
//include <Eigen/Geometry>
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
        force_sub_ = nh_.subscribe("/optoforce_node/OptoForceWrench", 1, &AdmittanceSubscriberClass::forceCallback, this);
        pose_sub_ = nh_.subscribe("/duck_purple/vrpn_client/estimated_odometry", 1, &AdmittanceSubscriberClass::poseCallback, this);
        odom_sub_ = nh_.subscribe("red/odometry", 1, &AdmittanceSubscriberClass::odomCallback, this);
        mod_trajectory_pub_= nh_.advertise<geometry_msgs::PoseStamped>("/red/tracker/input_pose", 10);
        filtered_sub = nh_.subscribe("/red/filtered_force", 1, &AdmittanceSubscriberClass::filtForceCb, this);
        set_k_sub_ = nh_.subscribe("/red/setK", 1, &AdmittanceSubscriberClass::setKCb, this); 
        force_filter_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/red/filtered_force", 10);
        xc_yc_pub_ = nh_.advertise<geometry_msgs::Vector3>("/red/position_cmd", 10);
        boolean_sub = nh_.subscribe<std_msgs::Bool>("/red/enable_admit_control", 1, &AdmittanceSubscriberClass::enableCb, this);

        HISTORY_BUFFER_SIZE = 40;

        pose_received = false;
        force_received = false;

        //trans matrix sensor->body
        Tws_ << 0, 0, -1, 0,
                0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;

        Tws_inv_ = Tws_.inverse();

        server.setCallback(boost::bind(&AdmittanceSubscriberClass::callback, this, _1, _2));
        

        
    }


    // Callback functions

    void enableCb(const std_msgs::Bool::ConstPtr& msg){
        enable_msg = msg->data; 
    }

    void filtForceCb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
        if(!calibrated){
            calibrate(msg, timeout);
        }
    }

    void setKCb (const std_msgs::Float32::ConstPtr& msg){
        K = msg->data;
        ROS_INFO_STREAM("K is set to:" << K );
    }
    
    void calibrate(const geometry_msgs::WrenchStamped::ConstPtr& msg, double timeout){
        ROS_INFO_THROTTLE(1, "Force calibration...");
        if(first_run){
            ros::Time start_time = ros::Time::now();
            calibration_start_time = start_time.toSec();
            elapsed = 0;
            first_run = false;
        }
        else{
            elapsed = ros::Time::now().toSec() - calibration_start_time;
        }


        if(elapsed < timeout){
            double force_x = msg->wrench.force.x;
            double force_y = msg->wrench.force.y;
            double force_z = msg->wrench.force.z;

            ff_x_calibrate.push_back(force_x);
            ff_y_calibrate.push_back(force_y);
            ff_z_calibrate.push_back(force_z);
        }
        else{
            calibrated_value.wrench.force.x = std::accumulate(ff_x_calibrate.begin(), ff_x_calibrate.end(), 0.0)/ff_x_calibrate.size();
            calibrated_value.wrench.force.y = std::accumulate(ff_y_calibrate.begin(), ff_y_calibrate.end(), 0.0)/ff_y_calibrate.size();
            calibrated_value.wrench.force.z = std::accumulate(ff_z_calibrate.begin(), ff_z_calibrate.end(), 0.0)/ff_z_calibrate.size();
            calibrated = true;
            ROS_INFO_STREAM("f_x: " << calibrated_value.wrench.force.x << "ff_y: " << calibrated_value.wrench.force.y << "ff_z: " << calibrated_value.wrench.force.z);

        }       
        
    }

    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {
        double force_x; double force_y; double force_z;
        //save data from the topic
        ROS_INFO_ONCE("Received force message");
        
        if(std::abs(msg->wrench.force.x)<0.1 ){
            force_x = 0;
        }
        else{
            force_x = msg->wrench.force.x;
        }
        if(std::abs(msg->wrench.force.y)<0.1 ){
            force_y = 0;
        }
        else{
            force_y = msg->wrench.force.y;
        }
        if(std::abs(msg->wrench.force.z)<0.1 ){
            force_z = 0;
        }
        else{
            force_z = msg->wrench.force.z;
        }

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

        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;
        current_z = msg->pose.pose.position.z;

    }

    void updatePose(const geometry_msgs::PoseStamped& pose) {
        std::lock_guard<std::mutex> lock(output_msg_mutex);
        m_output_msg = pose;
    }

    void publish() {
        std::lock_guard<std::mutex> lock(output_msg_mutex);
        mod_trajectory_pub_.publish(m_output_msg);
    }

    Eigen::Matrix3d quaternionToRotation(const Eigen::Quaterniond& q) {
        Eigen::Matrix3d rotMat = q.toRotationMatrix();
        return rotMat;
    }

    void callback(uam_force_control::parametersConfig& config, uint32_t level)
    {
        K = config.stiffness;
        ROS_INFO("Stiffness changed to: %f", K);
    }

    void run()
    {   ROS_INFO_STREAM("Enable msg: " << enable_msg << "Calibrated: " << calibrated);  
        if(enable_msg && calibrated){
        ROS_INFO("running");
        geometry_msgs::PoseStamped output_msg;
        geometry_msgs::Vector3 xc_yc_msg;

        //get the information from mavros/global_position/local
        double x = poseCbMsg.pose.pose.position.x;
        double y = poseCbMsg.pose.pose.position.y;
        double z = poseCbMsg.pose.pose.position.z;
        double qx = poseCbMsg.pose.pose.orientation.x;
        double qy = poseCbMsg.pose.pose.orientation.y;
        double qz = poseCbMsg.pose.pose.orientation.z;
        double qw = poseCbMsg.pose.pose.orientation.w;

        Eigen::Quaterniond quat(qw, qx, qy, qz);
        Eigen::Matrix3d rotMat = quaternionToRotation(quat);
        Eigen::Matrix4d tranMat;
        Eigen::Matrix4d tranMat_inv;
        tranMat.block<3, 3>(0, 0) = rotMat;
        tranMat.row(0)[3] = x;
        tranMat.row(1)[3] = y;
        tranMat.row(2)[3] = z;
        tranMat.row(3)[3] = 1;
        tranMat.row(3)[0] = 0;
        tranMat.row(3)[1] = 0;
        tranMat.row(3)[2] = 0;

        Eigen::IOFormat matrixFormat(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
        //ROS_INFO_STREAM("quat: " << qx << "\ " << qy << "\ " << qz << "\"); 
        ROS_INFO_STREAM("transformational matrix: \n" << tranMat.format(matrixFormat));

        tranMat_inv = tranMat.inverse();

        //just casting
        double xc = filtered_msg.wrench.force.x/K;
        double yc = filtered_msg.wrench.force.y/K;
        double zc = filtered_msg.wrench.force.z/K;

        //FORCE TRANSFORM WORLD->SENSOR FRAME

        //make a force vector
        Eigen::Vector4d force_vector;
        force_vector << xc, yc, zc, 1;

        //Eigen::Vector4d force_vector_body = Tws_inv_ * force_vector; 

        //multiplying the force vector with invers of a transformational matrix
        Eigen::Vector4d force_vector_transform = tranMat * Tws_ * force_vector;
        //transforming the vector back to geometry_msgs::WrenchStamped
        double xct = force_vector_transform(0);
        double yct = force_vector_transform(1);
        double zct = force_vector_transform(2);
        ROS_INFO_STREAM("Wx: " << xct << "Wy: " << yct << "Wz: " << zct); 

        //test publisher publishing
        xc_yc_msg.x = xct;
        xc_yc_msg.y = yct;
        xc_yc_msg.z = zct;
        xc_yc_pub_.publish(xc_yc_msg);

        // Modifying the data before publishing
        output_msg.pose.position.x = xct;  
        output_msg.pose.position.y = yct;
        output_msg.pose.position.z = z;
        output_msg.pose.orientation.w = qw;
        output_msg.pose.orientation.x = qx;
        output_msg.pose.orientation.y = qy;
        output_msg.pose.orientation.z = qz;

        // updatePose(output_msg);
        // publish();
        mod_trajectory_pub_.publish(output_msg); //publishing modified message

        //mod_trajectory_pub_.publish(output_msg); //publishing modified message
        pose_received = false;
        force_received = false;
    }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber force_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher mod_trajectory_pub_;
    ros::Publisher force_filter_pub_;
    ros::Publisher xc_yc_pub_;
    ros::Publisher boolean_pub;
    ros::Subscriber boolean_sub;
    ros::Subscriber filtered_sub;
    ros::Subscriber set_k_sub_;

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

    dynamic_reconfigure::Server<uam_force_control::parametersConfig> server;
    
    //node enable flag
    bool enable_msg = false;

    //calibration flag
    bool calibrated = false;

    //duration of calibration procedure
    double timeout = 5;

    //lists for calibration
    std::vector<double> ff_x_calibrate;
    std::vector<double> ff_y_calibrate;
    std::vector<double> ff_z_calibrate;

    geometry_msgs::WrenchStamped calibrated_value;

    bool first_run = true;
    double calibration_start_time;

    int elapsed;

    //current position callback function variables
    double current_x;
    double current_y;
    double current_z;
};


int main(int argc, char **argv)
{

    // Initialize the ROS node
    ros::init(argc, argv, "admittance_node");

    // Create an instance of the subscriber/publisher class
    AdmittanceSubscriberClass my_subscriber;

    ros::Rate rate(50);
    while(ros::ok()){
        my_subscriber.run();
        rate.sleep();
        ros::spinOnce(); 
    }

    ros::shutdown(); 

    // Spin the node
    //ros::spin();

    return 0;
}
