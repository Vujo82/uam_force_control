class AdmittanceSubscriberClass {
public:
    AdmittanceSubscriberClass();
    void callback(const std_msgs::String::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};
