/*
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MavRos node implementation class
 * @file mavros_router.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 *  @brief MAVROS node implementation
 */

#pragma once

#ifndef MAVROS__MAVROS_ROUTER_HPP_
#define MAVROS__MAVROS_ROUTER_HPP_

#include <array>
#include <memory>
#include <set>
#include <string>
#include <shared_mutex>     // NOLINT
#include <utility>
#include <vector>
#include <unordered_map>
#include <Eigen/Eigen>      // NOLINT

#include "mavconn/interface.hpp"
#include "mavconn/mavlink_dialect.hpp"
#include "mavros/utils.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include "mavros_msgs/msg/mavlink.hpp"
#include "mavros_msgs/srv/endpoint_add.hpp"
#include "mavros_msgs/srv/endpoint_del.hpp"

namespace mavros
{
namespace router
{

using id_t = uint32_t;
using addr_t = uint32_t;

using mavconn::Framing;
using ::mavlink::mavlink_message_t;
using ::mavlink::msgid_t;

using namespace std::placeholders;      // NOLINT
using namespace std::chrono_literals;   // NOLINT

class Router;

/**
 * Endpoint base class
 *
 * Represents one network connection to the Router
 *
 * One endpoint could map to several remote devices,
 * e.g. mesh radio for swarm.
 */
class Endpoint
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Endpoint)

  enum class Type
  {
    fcu = 0,
    gcs = 1,
    uas = 2,
  };

protected:
  explicit Endpoint(Router * router, uint32_t id, Type link_type, std::string url)
  : router {router}
  , id {id}
  , link_type {link_type}
  , url {url}
  {
    const addr_t broadcast_addr = 0;

    // Accept broadcasts by default
    remote_addrs.emplace(broadcast_addr);
  }

public:
  /**
   * @brief Destructor.
   *
   * Virtual destructor is required, because @a Router used polymorphic pointers to @a Endpoint.
   */
  virtual ~Endpoint() = default;

  virtual bool is_open() const = 0;
  virtual void reconnect() = 0;

  virtual void send_message(
    const mavlink_message_t * msg, const Framing framing = Framing::ok,
    id_t src_id = 0) = 0;
  virtual void recv_message(const mavlink_message_t * msg, const Framing framing = Framing::ok);

  virtual std::string diag_name();
  virtual void diag_run(diagnostic_updater::DiagnosticStatusWrapper & stat) = 0;

  Type get_link_type() const noexcept {
    return link_type;
  }

  std::string const& get_url() const noexcept {
    return url;
  }

  uint32_t get_id() const noexcept {
    return id;
  }

  bool has_remote_addr(addr_t addr) const noexcept {
    return remote_addrs.find(addr) != remote_addrs.end();
  }

  void clear_stale_remote_addrs();

protected:
  Router * const router;
  uint32_t const id;                         // id of the endpoint
  Type const link_type;                      // class of the endpoint
  std::string const url;                     // url to open that endpoint
  std::set<addr_t> remote_addrs;       // remotes that we heard there
  std::set<addr_t> stale_addrs;        // temporary storage for stale remote addrs
};

/**
 * Router class implements MAVROS Router node
 *
 * Router is essential part that connects several MAVLink devices together.
 * In general there are three device classes that router uses:
 * - a FCU endpoint - should be faced to autopilot firmware(s),
 * - a GCS endpoint - that ep connects control software
 * - a UAS endpoint - special type of endpoint that is used to connect MAVROS UAS node(s) to FCU(s)
 *
 * Some traffic rules:
 * 1. FCU broadcast -> GCS, UAS, but not to any other FCU
 * 2. FCU targeted system/component -> GCS/UAS endpoint that have matching address
 * 3. FCU targeted system -> same as for p.2
 * 4. GCS broadcast -> FCU, UAS
 * 5. GCS targeted -> FCU/UAS maching addr
 * 6. UAS broadcast -> FCU, GCS
 * 7. UAS targeted -> FCU/GCS
 */
class Router : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Router)

  using StrV = std::vector<std::string>;

  explicit Router(const std::string & node_name = "mavros_router")
  : Router(rclcpp::NodeOptions(), node_name) {}

  explicit Router(
    const rclcpp::NodeOptions & options,
    const std::string & node_name = "mavros_router");

  Router(Router const &) = delete;
  Router(Router &&) = delete;

  ~Router();

  void route_message(Endpoint& src, const mavlink_message_t * msg, const Framing framing);

private:
  friend class Endpoint;
  friend class TestRouter;

  static std::atomic<id_t> id_counter;

  std::shared_timed_mutex mu;

  // map stores all routing endpoints
  std::unordered_map<id_t, Endpoint::UniquePtr> endpoints;

  std::atomic<size_t> stat_msg_routed;      //!< amount of messages came to route_messages()
  std::atomic<size_t> stat_msg_sent;        //!< amount of messages sent
  std::atomic<size_t> stat_msg_dropped;     //!< amount of messages dropped

  rclcpp::Service<mavros_msgs::srv::EndpointAdd>::SharedPtr add_service;
  rclcpp::Service<mavros_msgs::srv::EndpointDel>::SharedPtr del_service;
  rclcpp::TimerBase::SharedPtr reconnect_timer;
  rclcpp::TimerBase::SharedPtr stale_addrs_timer;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr set_parameters_handle_ptr;
  diagnostic_updater::Updater diagnostic_updater;

  void add_endpoint(
    const mavros_msgs::srv::EndpointAdd::Request::SharedPtr request,
    mavros_msgs::srv::EndpointAdd::Response::SharedPtr response);
  void del_endpoint(
    const mavros_msgs::srv::EndpointDel::Request::SharedPtr request,
    mavros_msgs::srv::EndpointDel::Response::SharedPtr response);

  void periodic_reconnect_endpoints();
  void periodic_clear_stale_remote_addrs();

  rcl_interfaces::msg::SetParametersResult on_set_parameters_cb(
    const std::vector<rclcpp::Parameter> & parameters);

  void diag_run(diagnostic_updater::DiagnosticStatusWrapper & stat);
  std::set<std::string> get_existing_set(Endpoint::Type type);
  void update_endpoints(const rclcpp::Parameter & parameter, Endpoint::Type type);
};

/**
 * MAVConnEndpoint implements Endpoint for FCU or GCS connection
 * via good old libmavconn url's
 *
 * TODO(vooon): support multiple remotes on UDP,
 *              choose right TCP client instead of simple broadcast
 *
 * NOTE(vooon): do we still need PX4 USB quirk?
 */
class MAVConnEndpoint : public Endpoint
{
public:
  explicit MAVConnEndpoint(Router * router, uint32_t id, Type link_type, std::string url);
  ~MAVConnEndpoint();

private:
  mavconn::MAVConnInterface::Ptr link;       // connection
  size_t stat_last_drop_count;

  bool is_open() const override;
  void reconnect() override;
  void open();
  void close();

  void send_message(
    const mavlink_message_t * msg, const Framing framing = Framing::ok,
    id_t src_id = 0) override;

  void diag_run(diagnostic_updater::DiagnosticStatusWrapper & stat) override;
};

/**
 * ROSEndpoint implements Endpoint for UAS node
 *
 * That endpoint converts mavlink messages to ROS2 IDL
 * and passes them trough DDL messaging or intra-process comms.
 *
 * Each drone would have separate UAS node
 */
class ROSEndpoint : public Endpoint
{
public:
  explicit ROSEndpoint(Router * router, uint32_t id, Type link_type, std::string url);
  ~ROSEndpoint();

private:
  rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr sink;       // UAS -> FCU
  rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr source;        // FCU -> UAS

  bool is_open() const override;
  void reconnect() override;
  void open();
  void close();

  void send_message(
    const mavlink_message_t * msg, const Framing framing = Framing::ok,
    id_t src_id = 0) override;

  void diag_run(diagnostic_updater::DiagnosticStatusWrapper & stat) override;

private:
  void ros_recv_message(const mavros_msgs::msg::Mavlink::SharedPtr rmsg);
};

}   // namespace router
}   // namespace mavros

#endif  // MAVROS__MAVROS_ROUTER_HPP_
