#ifndef SUBSCRIBER_XBOT_GPS_H
#define SUBSCRIBER_XBOT_GPS_H
/**************************
 *      WOLF includes     *
 **************************/
#include <core/yaml/parser_yaml.h>
#include <core/common/wolf.h>
#include <core/problem/problem.h>
#include <core/utils/params_server.h>
#include <gnss/capture/capture_gnss_fix.h>
#include <gnss/sensor/sensor_gnss.h>

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <xbot_msgs/AbsolutePose.h>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <iomanip>
#include <queue>

/**************************
 *    WOLF-ROS includes   *
 **************************/
#include "subscriber.h"

/**************************
 *     Other includes     *
 **************************/
#include "gnss_utils/gnss_utils.h"

namespace wolf
{

class SubscriberXbotGps : public Subscriber
{
        std::string cov_mode_;
        double cov_factor_;
        double cov_min_;
        Eigen::Matrix3d cov_;

    public:
        // Constructor
        SubscriberXbotGps(const std::string& _unique_name,
                          const ParamsServer& _server,
                          const SensorBasePtr _sensor_ptr);
        WOLF_SUBSCRIBER_CREATE(SubscriberXbotGps);

        virtual void initialize(ros::NodeHandle& nh, const std::string& topic);

        void callback(const xbot_msgs::AbsolutePose::ConstPtr& msg);
};
WOLF_REGISTER_SUBSCRIBER(SubscriberXbotGps)

}
#endif