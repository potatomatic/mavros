/**
 * @brief EKF status plugin
 * @file ekf_status.cpp
 * @author ugol-1 <muzhyk.belarus@protonmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>

#include <mavros_msgs/msg/ekf_status_report.hpp>

#include <mavlink/v2.0/ardupilotmega/ardupilotmega.hpp>

#include <rcpputils/asserts.hpp>

#include <string>

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT
using mavlink::common::RTK_BASELINE_COORDINATE_SYSTEM;

/**
 * @brief Mavlink EKF status plugin.
 * @plugin ekf_status
 *
 * This plugin publishes EKF status from a Mavlink compatible FCU to ROS.
 */
class EKFStatusPlugin : public plugin::Plugin
{
public:
  explicit EKFStatusPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "ekf_status")
  , ekf_status_report_publisher_ {node->create_publisher<mavros_msgs::msg::EKFStatusReport>("~/ekf_status_report", rclcpp::SensorDataQoS {})}
  {
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&EKFStatusPlugin::handle_ekf_status_report)
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::EKFStatusReport>::SharedPtr ekf_status_report_publisher_;

  void handle_ekf_status_report(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::ardupilotmega::msg::EKF_STATUS_REPORT& mav_msg,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    mavros_msgs::msg::EKFStatusReport ros_msg;

    ros_msg.header.stamp = node->now();
    ros_msg.flags = mav_msg.flags;
    ros_msg.velocity_variance = mav_msg.velocity_variance;
    ros_msg.pos_horiz_variance = mav_msg.pos_horiz_variance;
    ros_msg.pos_vert_variance = mav_msg.pos_vert_variance;
    ros_msg.compass_variance = mav_msg.compass_variance;
    ros_msg.terrain_alt_variance = mav_msg.terrain_alt_variance;
    ros_msg.airspeed_variance = mav_msg.airspeed_variance;

    ekf_status_report_publisher_->publish(ros_msg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::EKFStatusPlugin)
