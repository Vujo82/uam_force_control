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

//#include <AdmittanceControl.h>

//AdmittanceSubscriberClass - klasa za ocitavanje subscribera

class AdmittanceSubscriberClass
{
public:
    AdmittanceSubscriberClass()
    {
        // Create a node handle
        nh_ = ros::NodeHandle();

        // Subscribe to the topic
        force_sub_ = nh_.subscribe("red/ft_sensor", 1, &AdmittanceSubscriberClass::forceCallback, this);
        pose_sub_ = nh_.subscribe("/red/position_hold/trajectory", 1, &AdmittanceSubscriberClass::poseCallback, this);
        odom_sub_ = nh_.subscribe("red/odometry", 1, &AdmittanceSubscriberClass::odomCallback, this);
        mod_trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/red/custom_pub", 10);

    }

    // Callback functions
    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {
        //ROS_INFO("Received force message: force.x = %f, force.y = %f, force.z = %f, torque.x = %f, torque.y = %f, torque.z = %f", msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
        ROS_INFO("Received force message:");
    }

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
};


//DroneController - klasa za racunanje dif funkcije sustava i racunanje modificirane trajektorije

// class DroneController
// {
// public:
//     DroneController(double M, double D, double K) : M_(M), D_(D), K_(K) {}
    
//     void setDesiredTrajectory(const trajectory_msgs::MultiDOFJointTrajectoryPoint& d) {
//         d_ = d;
//     }
    
//     void setExternalForces(const geometry_msgs::WrenchStamped& f) {
//         f_ = f;
//     }
    
//     trajectory_msgs::MultiDOFJointTrajectory getModifiedTrajectory(const trajectory_msgs::MultiDOFJointTrajectoryPoint& r0, double dt, double t_end) const {

//         // Initialize modified trajectory with initial values
//         trajectory_msgs::MultiDOFJointTrajectory r;
//         r.points.push_back(r0);
        
//         // Compute number of time steps
//         int n_steps = static_cast<int>(std::ceil(t_end / dt));
        
//         // Initialize vectors to store intermediate values
//         std::vector<double> r_dot(n_steps), r_dot_dot(n_steps);
        
//         // Compute modified trajectory at each time step using numerical integration
//         for (int i = 0; i < n_steps; ++i) {
//             // Compute intermediate values
//             double r_dot_dot_x = (f_.wrench.force.x - D_ * (r.points[i].velocities[0].linear.x - d_.velocities[0].linear.x) - K_ * (r.points[i].transforms[0].translation.x - d_.transforms[0].translation.x)) / M_;
//             double r_dot_dot_y = (f_.wrench.force.y - D_ * (r.points[i].velocities[0].linear.y - d_.velocities[0].linear.y) - K_ * (r.points[i].transforms[0].translation.y - d_.transforms[0].translation.y)) / M_;
//             double r_dot_dot_z = (f_.wrench.force.z - D_ * (r.points[i].velocities[0].linear.z - d_.velocities[0].linear.z) - K_ * (r.points[i].transforms[0].translation.z - d_.transforms[0].translation.z)) / M_;

//             trajectory_msgs::MultiDOFJointTrajectoryPoint r_next;
//             r_next.time_from_start = ros::Duration(dt * (i + 1));
//             r_next.transforms.resize(1);
//             r_next.transforms[0].translation.x = r.points[i].transforms[0].translation.x + r.points[i].velocities[0].linear.x * dt;
//             r_next.transforms[0].translation.y = r.points[i].transforms[0].translation.y + r.points[i].velocities[0].linear.y * dt;
//             r_next.transforms[0].translation.z = r.points[i].transforms[0].translation.z + r.points[i].velocities[0].linear.z * dt;
//             r_next.velocities.resize(1);
//             r_next.velocities[0].linear.x = r.points[i].velocities[0].linear.x + r_dot_dot_x * dt;
//             r_next.velocities[0].linear.y = r.points[i].velocities[0].linear.y + r_dot_dot_y * dt;
//             r_next.velocities[0].linear.z = r.points[i].velocities[0].linear.z + r_dot_dot_z * dt;

//             r.points.push_back(r_next);
//         }
        
//         return r;
//     }
    
// private:
//     double M_, D_, K_;
//     trajectory_msgs::MultiDOFJointTrajectoryPoint d_;
//     geometry_msgs::WrenchStamped f_;
// };



int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "admittance_node");

    // Create an instance of the subscriber/publisher class
    AdmittanceSubscriberClass my_subscriber; //mozda promijeniti ime instance klase al nije tolko vazno

    // Spin the node
    ros::spin();

    return 0;
}
