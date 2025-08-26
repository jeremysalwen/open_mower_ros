#include "../include/subscriber_xbot_gps.h"

namespace wolf
{

// Constructor
SubscriberXbotGps::SubscriberXbotGps(const std::string& _unique_name,
                                     const ParamsServer& _server,
                                     const SensorBasePtr _sensor_ptr)
    : Subscriber(_unique_name, _server, _sensor_ptr)
    , cov_factor_(1)
{

    cov_mode_ = _server.getParam<std::string>(prefix_ + "/cov_mode");
    cov_min_  = getParamWithDefault<double>   (_server, prefix_ + "/cov_min", 1e-6);

    if (cov_mode_ == "manual")
        cov_ = _server.getParam<Eigen::Matrix3d>(prefix_ + "/cov");
    else if (cov_mode_ == "factor")
        cov_factor_ = _server.getParam<double>(prefix_ + "/cov_factor");
    else if (cov_mode_ == "msg")
        cov_ = Eigen::Matrix3d::Identity(); // Initialize for msg mode
}


void SubscriberXbotGps::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    sub_     = nh.subscribe(topic, 1e3, &SubscriberXbotGps::callback, this);
}

void SubscriberXbotGps::callback(const xbot_msgs::AbsolutePose::ConstPtr& msg)
{
    updateLastHeader(msg->header);

    // Only process GPS source messages
    if (msg->source != xbot_msgs::AbsolutePose::SOURCE_GPS)
        return;

    if (cov_mode_ == "msg" or cov_mode_ == "factor")
    {
        // Extract 3x3 covariance from 6x6 pose covariance (position part only)
        Eigen::Matrix3d pose_cov;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                pose_cov(i,j) = msg->pose.covariance[i*6 + j];
        
        cov_ = cov_factor_ * pose_cov;
    }

    // min cov diagonal values
    for (int i = 0; i < 3; ++i)
        if (cov_(i, i) < cov_min_)
            cov_(i, i) = cov_min_;
    
    // Cov fix has the 4th element being clock bias (no information about this)
    Matrix4d cov_fix = Matrix4d::Identity() * 0.1;
    cov_fix.topLeftCorner<3,3>() = cov_;
    
    CaptureGnssFixPtr cap_gnss_ptr = std::make_shared<CaptureGnssFix>(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                                      sensor_ptr_,
                                                                      Eigen::Vector4d(msg->pose.pose.position.x,
                                                                                      msg->pose.pose.position.y,
                                                                                      msg->pose.pose.position.z,
                                                                                      0),
                                                                      cov_fix,
                                                                      false); // false = {ENU coordinates}
    cap_gnss_ptr->process();
}

}