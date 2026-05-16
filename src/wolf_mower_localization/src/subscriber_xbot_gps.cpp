#include "../include/subscriber_xbot_gps.h"

#include <GeographicLib/Geocentric.hpp>

namespace wolf {

// Constructor
SubscriberXbotGps::SubscriberXbotGps(const std::string& _unique_name, const ParamsServer& _server,
                                     const SensorBasePtr _sensor_ptr)
    : Subscriber(_unique_name, _server, _sensor_ptr),
      cov_factor_(1),
      datum_lat_(0.0),
      datum_lon_(0.0),
      datum_height_(0.0) {
  cov_mode_ = _server.getParam<std::string>(prefix_ + "/cov_mode");
  cov_min_ = getParamWithDefault<double>(_server, prefix_ + "/cov_min", 1e-6);

  if (cov_mode_ == "manual")
    cov_ = _server.getParam<Eigen::Matrix3d>(prefix_ + "/cov");
  else if (cov_mode_ == "factor")
    cov_factor_ = _server.getParam<double>(prefix_ + "/cov_factor");
  else if (cov_mode_ == "msg")
    cov_ = Eigen::Matrix3d::Identity();  // Initialize for msg mode
}

void SubscriberXbotGps::initialize(ros::NodeHandle& nh, const std::string& topic) {
  nh_ = nh;
  sub_ = nh.subscribe(topic, 1e3, &SubscriberXbotGps::callback, this);

  // Read GPS datum parameters from ROS parameter server
  nh.param("/xbot_driver_gps/datum_lat", datum_lat_, 0.0);
  nh.param("/xbot_driver_gps/datum_long", datum_lon_, 0.0);
  nh.param("/xbot_driver_gps/datum_height", datum_height_, 0.0);

  WOLF_INFO("GPS Datum: lat=", datum_lat_, ", lon=", datum_lon_, ", height=", datum_height_);
}

void SubscriberXbotGps::callback(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
  updateLastHeader(msg->header);

  // Only process GPS source messages
  if (msg->source != xbot_msgs::AbsolutePose::SOURCE_GPS) return;

  if (cov_mode_ == "msg" or cov_mode_ == "factor") {
    // Extract 3x3 covariance from 6x6 pose covariance (position part only)
    Eigen::Matrix3d pose_cov;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) pose_cov(i, j) = msg->pose.covariance[i * 6 + j];

    cov_ = cov_factor_ * pose_cov;
  }

  // min cov diagonal values
  for (int i = 0; i < 3; ++i)
    if (cov_(i, i) < cov_min_) cov_(i, i) = cov_min_;

  double datum_utm_n, datum_utm_e;
  std::string utm_zone;
  RobotLocalization::NavsatConversions::LLtoUTM(datum_lat_, datum_lon_, datum_utm_n, datum_utm_e, utm_zone);

  double absolute_utm_n = msg->pose.pose.position.y + datum_utm_n;
  double absolute_utm_e = msg->pose.pose.position.x + datum_utm_e;
  double absolute_alt = msg->pose.pose.position.z + datum_height_;

  double absolute_lat, absolute_lon;
  RobotLocalization::NavsatConversions::UTMtoLL(absolute_utm_n, absolute_utm_e, utm_zone, absolute_lat, absolute_lon);

  const auto& earth = GeographicLib::Geocentric::WGS84();

  double X, Y, Z;
  std::vector<double> M(9, 0.0);
  earth.Forward(absolute_lat, absolute_lon, absolute_alt, X, Y, Z, M);

  Eigen::Matrix3d R_enu2ecef;
  R_enu2ecef << M[0], M[1], M[2], M[3], M[4], M[5], M[6], M[7], M[8];

  Eigen::Matrix3d cov_ecef = R_enu2ecef * cov_ * R_enu2ecef.transpose();

  Matrix4d cov_fix = Matrix4d::Identity() * 0.1;
  cov_fix.topLeftCorner<3, 3>() = cov_ecef;

  CaptureGnssFixPtr cap_gnss_ptr = std::make_shared<CaptureGnssFix>(
      TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec), sensor_ptr_, Eigen::Vector4d(X, Y, Z, 0.0), cov_fix,
      true  // true = ECEF coordinates
  );
  cap_gnss_ptr->process();
}

}  // namespace wolf
