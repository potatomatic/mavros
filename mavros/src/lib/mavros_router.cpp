/*
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Mavros Router class
 * @file mavros_router.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#include <memory>
#include <vector>
#include <string>
#include <set>
#include <utility>

#include "mavros/mavros_router.hpp"
#include "rcpputils/asserts.hpp"

using namespace mavros::router;  // NOLINT
using rclcpp::QoS;

using unique_lock = std::unique_lock<std::shared_timed_mutex>;
using shared_lock = std::shared_lock<std::shared_timed_mutex>;

std::atomic<id_t> Router::id_counter {1000};

static inline uint8_t get_msg_byte(const mavlink_message_t * msg, uint8_t offset)
{
  return _MAV_PAYLOAD(msg)[offset];
}

Router::Router(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node {node_name, options}
, stat_msg_routed {0}
, stat_msg_sent {0}
, stat_msg_dropped {0}
, diagnostic_updater {this, 1.0}
{
  RCLCPP_DEBUG(this->get_logger(), "Start mavros::router::Router initialization...");

  add_service = this->create_service<mavros_msgs::srv::EndpointAdd>(
    "~/add_endpoint",
    std::bind(&Router::add_endpoint, this, _1, _2));
  del_service = this->create_service<mavros_msgs::srv::EndpointDel>(
    "~/del_endpoint",
    std::bind(&Router::del_endpoint, this, _1, _2));

  // try to reconnect endpoint each 30 seconds
  reconnect_timer =
    this->create_wall_timer(30s, std::bind(&Router::periodic_reconnect_endpoints, this));

  // collect garbage addrs each minute
  stale_addrs_timer =
    this->create_wall_timer(60s, std::bind(&Router::periodic_clear_stale_remote_addrs, this));

  diagnostic_updater.setHardwareID("none");  // NOTE: router connects several hardwares
  diagnostic_updater.add("MAVROS Router", this, &Router::diag_run);

  std::stringstream ss;
  for (auto & s : mavconn::MAVConnInterface::get_known_dialects()) {
    ss << " " << s;
  }

  RCLCPP_INFO(get_logger(), "Built-in SIMD instructions: %s", Eigen::SimdInstructionSetsInUse());
  RCLCPP_INFO(get_logger(), "Built-in MAVLink package version: %s", mavlink::version);
  RCLCPP_INFO(get_logger(), "Known MAVLink dialects:%s", ss.str().c_str());

  set_parameters_handle_ptr =
    this->add_on_set_parameters_callback(std::bind(&Router::on_set_parameters_cb, this, _1));

  // This effectively triggers the on_set_parameters_cb() and adds endpoints, if any.
  // We purposly do it at the very end of the Router() ctor, when all other parts of the Router object are initialized.
  this->declare_parameter<StrV>("fcu_urls", StrV());
  this->declare_parameter<StrV>("gcs_urls", StrV());
  this->declare_parameter<StrV>("uas_urls", StrV());

  RCLCPP_INFO(get_logger(), "MAVROS Router started");
}

Router::~Router()
{
  // Clear the endpoints before the Router data members get destroyed.
  // This ensures that threads created by endpoints are stopped before the Router parts get destroyed.
  endpoints.clear();
}

void Router::route_message(
  Endpoint& src, const mavlink_message_t * msg,
  const Framing framing)
{
  shared_lock lock(mu);
  this->stat_msg_routed++;

  // find message destination target
  addr_t target_addr = 0;
  auto msg_entry = ::mavlink::mavlink_get_msg_entry(msg->msgid);
  if (msg_entry) {
    if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM) {
      target_addr |= get_msg_byte(msg, msg_entry->target_system_ofs) << 8;
    }
    if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT) {
      target_addr |= get_msg_byte(msg, msg_entry->target_component_ofs);
    }
  }

  size_t sent_cnt = 0, retry_cnt = 0;
retry:
  for (auto & kv : this->endpoints) {
    auto & dest = kv.second;

    if (src.get_id() == dest->get_id()) {
      continue;     // do not echo message
    }
    if (src.get_link_type() == dest->get_link_type()) {
      continue;     // drop messages between same type FCU/GCS/UAS
    }

    // NOTE(vooon): current router do not allow to speak drone-to-drone.
    //              if it is needed perhaps better to add mavlink-router in front of mavros-router.

    bool const has_target = dest->has_remote_addr(target_addr);

    if (has_target) {
      dest->send_message(msg, framing, src.get_id());
      sent_cnt++;
    }
  }

  // if message haven't been sent retry broadcast it
  if (sent_cnt == 0 && retry_cnt < 2) {
    target_addr = 0;
    retry_cnt++;
    goto retry;
  }

  // update stats
  this->stat_msg_sent.fetch_add(sent_cnt);
  if (sent_cnt == 0) {
    this->stat_msg_dropped++;

    auto lg = get_logger();
    auto clock = get_clock();

    RCLCPP_WARN_THROTTLE(
      lg,
      *clock, 10000, "Message dropped: msgid: %d, source: %d.%d, target: %d.%d", msg->msgid,
      msg->sysid, msg->compid, target_addr >> 8,
      target_addr & 0xff);
  }
}

void Router::add_endpoint(
  const mavros_msgs::srv::EndpointAdd::Request::SharedPtr request,
  mavros_msgs::srv::EndpointAdd::Response::SharedPtr response)
{
  unique_lock lock(mu);
  auto lg = get_logger();

  RCLCPP_INFO(
    lg, "Requested to add endpoint: type: %d, url: %s", request->type,
    request->url.c_str());

  if (request->type > mavros_msgs::srv::EndpointAdd::Request::TYPE_UAS) {
    RCLCPP_ERROR(lg, "Unknown endpoint type");
    response->successful = false;
    response->reason = "unknown link type";
    return;
  }

  id_t const id = id_counter.fetch_add(1);

  try
  {
    Endpoint::UniquePtr ep;
    if (request->type == mavros_msgs::srv::EndpointAdd::Request::TYPE_UAS) {
      ep = std::make_unique<ROSEndpoint>(this, id, static_cast<Endpoint::Type>(request->type), request->url);
    } else {
      ep = std::make_unique<MAVConnEndpoint>(this, id, static_cast<Endpoint::Type>(request->type), request->url);
    }

    RCLCPP_INFO(lg, "Endpoint link[%d] created", id);

    this->endpoints[id] = std::move(ep);
    response->successful = true;
    response->id = id;
  } catch (std::runtime_error const & e) {
    RCLCPP_ERROR_STREAM(lg, "link [" << id << "] open failed: " << e.what());
    response->successful = false;
    response->reason = e.what();
  }
}

void Router::del_endpoint(
  const mavros_msgs::srv::EndpointDel::Request::SharedPtr request,
  mavros_msgs::srv::EndpointDel::Response::SharedPtr response)
{
  unique_lock lock(mu);
  auto lg = get_logger();

  if (request->id != 0) {
    RCLCPP_INFO(lg, "Requested to del endpoint id: %d", request->id);
    auto it = this->endpoints.find(request->id);
    if (it != this->endpoints.end() ) {
      this->endpoints.erase(it);
      response->successful = true;
    }
    return;
  }

  RCLCPP_INFO(
    lg, "Requested to del endpoint type: %d url: %s", request->type,
    request->url.c_str());
  for (auto it = this->endpoints.cbegin(); it != this->endpoints.cend(); it++) {
    if (it->second->get_url() == request->url &&
      it->second->get_link_type() == static_cast<Endpoint::Type>( request->type))
    {
      this->endpoints.erase(it);
      response->successful = true;
      return;
    }
  }
}

rcl_interfaces::msg::SetParametersResult Router::on_set_parameters_cb(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto lg = get_logger();
  rcl_interfaces::msg::SetParametersResult result{};

  RCLCPP_DEBUG(lg, "params callback");

  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto name = parameter.get_name();
    if (name == "fcu_urls") {
      update_endpoints(parameter, Endpoint::Type::fcu);
    } else if (name == "gcs_urls") {
      update_endpoints(parameter, Endpoint::Type::gcs);
    } else if (name == "uas_urls") {
      update_endpoints(parameter, Endpoint::Type::uas);
    } else {
      result.successful = false;
      result.reason = "unknown parameter";
    }
  }

  return result;
}

std::set<std::string> Router::get_existing_set(Endpoint::Type type) {
  shared_lock lock(this->mu);

  std::set<std::string> ret;

  for (const auto & kv : this->endpoints) {
    if (kv.second->get_link_type() != type) {
      continue;
    }

    ret.emplace(kv.second->get_url());
  }

  return ret;
};


void Router::update_endpoints(const rclcpp::Parameter & parameter, Endpoint::Type type) {
  RCLCPP_DEBUG(get_logger(), "Processing urls parameter: %s", parameter.get_name().c_str());

  auto urls = parameter.as_string_array();
  std::set<std::string> urls_set(urls.begin(), urls.end());
  auto existing_set = get_existing_set(type);

  std::set<std::string> to_add{}, to_del{};
  std::set_difference(
    urls_set.begin(), urls_set.end(), existing_set.begin(),
    existing_set.end(), std::inserter(to_add, to_add.begin()));
  std::set_difference(
    existing_set.begin(), existing_set.end(), urls_set.begin(),
    urls_set.end(), std::inserter(to_del, to_del.begin()));

  for (auto url : to_add) {
    auto req = std::make_shared<mavros_msgs::srv::EndpointAdd::Request>();
    auto resp = std::make_shared<mavros_msgs::srv::EndpointAdd::Response>();

    req->type = utils::enum_value(type);
    req->url = url;

    this->add_endpoint(req, resp);
  }

  for (auto url : to_del) {
    auto req = std::make_shared<mavros_msgs::srv::EndpointDel::Request>();
    auto resp = std::make_shared<mavros_msgs::srv::EndpointDel::Response>();

    req->type = utils::enum_value(type);
    req->url = url;

    this->del_endpoint(req, resp);
  }
};

void Router::periodic_reconnect_endpoints()
{
  shared_lock lock(mu);
  auto lg = get_logger();

  RCLCPP_DEBUG(lg, "start reconnecting...");

  for (auto & kv : this->endpoints) {
    auto & p = kv.second;

    if (p->is_open()) {
      continue;
    }

    RCLCPP_INFO(lg, "link[%d] trying to reconnect...", p->get_id());

    try
    {
      p->reconnect();
      RCLCPP_INFO_STREAM(lg, "link[" << p->get_id() << "] reconnected");
    } catch (std::runtime_error const & e)
    {
      RCLCPP_ERROR_STREAM(lg, "link[" << p->get_id() << "] reconnect failed: " << e.what());
    }
  }
}

void Router::periodic_clear_stale_remote_addrs()
{
  unique_lock lock(mu);
  auto lg = get_logger();

  RCLCPP_DEBUG(lg, "clear stale remotes");
  for (auto & kv : this->endpoints) {
    kv.second->clear_stale_remote_addrs();
  }
}

void Endpoint::clear_stale_remote_addrs()
{
  // Step 1: remove any stale addrs that still there
  //         (hadn't been removed by Endpoint::recv_message())
  for (auto addr : stale_addrs) {
    if (addr != 0) {
      remote_addrs.erase(addr);
      RCLCPP_INFO_STREAM(
        router->get_logger(), "link[" << id << "] removed stale remote address " << (addr >> 8) << "." << (addr & 0xff));
    }
  }

  // Step 2: re-initiate stale_addrs
  stale_addrs = remote_addrs;
}

void Router::diag_run(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  auto endpoints_len = [this]() -> auto {
      shared_lock lock(this->mu);
      return this->endpoints.size();
    } ();

  stat.addf("Endpoints", "%zu", endpoints_len);
  stat.addf("Messages routed", "%zu", stat_msg_routed.load());
  stat.addf("Messages sent", "%zu", stat_msg_sent.load());
  stat.addf("Messages dropped", "%zu", stat_msg_dropped.load());

  if (endpoints_len < 2) {
    stat.summary(2, "not enough endpoints");
  } else {
    stat.summary(0, "ok");
  }
}

void Endpoint::recv_message(const mavlink_message_t * msg, const Framing framing)
{
  rcpputils::assert_true(msg, "msg not nullptr");
  rcpputils::assert_true(this->router, "router not nullptr");

  const addr_t sysid_addr = msg->sysid << 8;
  const addr_t sysid_compid_addr = (msg->sysid << 8) | msg->compid;

  // save source addr to remote_addrs
  auto sp = this->remote_addrs.emplace(sysid_addr);
  auto scp = this->remote_addrs.emplace(sysid_compid_addr);

  // and delete it from stale_addrs
  this->stale_addrs.erase(sysid_addr);
  this->stale_addrs.erase(sysid_compid_addr);

  auto & nh = this->router;
  if (sp.second || scp.second) {
    RCLCPP_INFO(
      nh->get_logger(), "link[%d] detected remote address %d.%d", this->id, msg->sysid,
      msg->compid);
  }

  nh->route_message(*this, msg, framing);
}

Endpoint::Endpoint(Router * router, uint32_t id, Type link_type, std::string url)
: router {router}
, id {id}
, link_type {link_type}
, url {std::move(url)}
{
  const addr_t broadcast_addr = 0;

  // Accept broadcasts by default
  remote_addrs.emplace(broadcast_addr);

  this->router->diagnostic_updater.add(diag_name(), std::bind(&Endpoint::diag_run, this, _1));
}

Endpoint::~Endpoint()
{
  this->router->diagnostic_updater.removeByName(diag_name());
}

std::string Endpoint::diag_name() const
{
  return utils::format("endpoint %d: %s", this->id, this->url.c_str());
}

MAVConnEndpoint::MAVConnEndpoint(Router * router, uint32_t id, Type link_type, std::string url)
: Endpoint {router, id, link_type, url}
, stat_last_drop_count {0}
{
  open();
}

MAVConnEndpoint::~MAVConnEndpoint()
{
  close();
}

bool MAVConnEndpoint::is_open() const
{
  if (!this->link) {
    return false;
  }

  return this->link->is_open();
}

void MAVConnEndpoint::reconnect()
{
  open();
}

void MAVConnEndpoint::open()
{
  this->link = mavconn::MAVConnInterface::open_url(
    this->url, 1, mavconn::MAV_COMP_ID_UDP_BRIDGE, std::bind(
      &MAVConnEndpoint::recv_message,
      this, _1, _2));

  // not necessary because router would not serialize mavlink::Message
  // but that is a good default
  this->link->set_protocol_version(mavconn::Protocol::V20);

  // TODO(vooon): message signing?
}

void MAVConnEndpoint::close()
{
  if (!this->link) {
    return;
  }

  this->link->close();
  this->link.reset();
}

void MAVConnEndpoint::send_message(
  const mavlink_message_t * msg, const Framing framing,
  id_t src_id [[maybe_unused]])
{
  (void)framing;

  if (!this->link) {
    return;
  }

  this->link->send_message_ignore_drop(msg);
}

void MAVConnEndpoint::diag_run(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!this->link) {
    stat.summary(2, "closed");
    return;
  }

  auto mav_status = this->link->get_status();
  auto iostat = this->link->get_iostat();

  stat.addf("Received packets", "%u", mav_status.packet_rx_success_count);
  stat.addf("Dropped packets", "%u", mav_status.packet_rx_drop_count);
  stat.addf("Buffer overruns", "%u", mav_status.buffer_overrun);
  stat.addf("Parse errors", "%u", mav_status.parse_error);
  stat.addf("Rx sequence number", "%u", mav_status.current_rx_seq);
  stat.addf("Tx sequence number", "%u", mav_status.current_tx_seq);

  stat.addf("Rx total bytes", "%u", iostat.rx_total_bytes);
  stat.addf("Tx total bytes", "%u", iostat.tx_total_bytes);
  stat.addf("Rx speed", "%f", iostat.rx_speed);
  stat.addf("Tx speed", "%f", iostat.tx_speed);

  stat.addf("Remotes count", "%zu", this->remote_addrs.size());
  size_t idx = 0;
  for (auto addr : this->remote_addrs) {
    stat.addf(utils::format("Remote [%d]", idx++), "%d.%d", addr >> 8, addr & 0xff);
  }

  if (mav_status.packet_rx_drop_count > stat_last_drop_count) {
    stat.summaryf(
      1, "%d packeges dropped since last report",
      mav_status.packet_rx_drop_count - stat_last_drop_count);
  } else {
    stat.summary(0, "ok");
  }

  stat_last_drop_count = mav_status.packet_rx_drop_count;
}

ROSEndpoint::ROSEndpoint(Router * router, uint32_t id, Type link_type, std::string url)
: Endpoint {router, id, link_type, url}
{
  open();
}

ROSEndpoint::~ROSEndpoint()
{
  close();
}

bool ROSEndpoint::is_open() const
{
  return this->source && this->sink;
}

void ROSEndpoint::reconnect()
{
  open();
}

void ROSEndpoint::open()
{
  auto & nh = this->router;
  if (!nh) {
    throw std::logic_error {"router not set"};
  }

  auto qos = QoS(
    1000).best_effort().durability_volatile();
  this->source =
    nh->create_publisher<mavros_msgs::msg::Mavlink>(
    utils::format(
      "%s/%s", this->url.c_str(),
      "mavlink_source"), qos);
  this->sink = nh->create_subscription<mavros_msgs::msg::Mavlink>(
    utils::format("%s/%s", this->url.c_str(), "mavlink_sink"), qos,
    std::bind(&ROSEndpoint::ros_recv_message, this, _1));
}

void ROSEndpoint::close()
{
  this->source.reset();
  this->sink.reset();
}

void ROSEndpoint::send_message(const mavlink_message_t * msg, const Framing framing, id_t src_id)
{
  rcpputils::assert_true(msg, "msg not null");

  auto rmsg = mavros_msgs::msg::Mavlink();
  auto ok = mavros_msgs::mavlink::convert(*msg, rmsg, utils::enum_value(framing));

  // don't fail if endpoint closed
  if (!this->source) {
    return;
  }

  rmsg.header.stamp = this->router->now();
  rmsg.header.frame_id = utils::format("ep:%d", src_id);

  if (ok) {
    this->source->publish(rmsg);
  } else if (auto & nh = this->router) {
    RCLCPP_ERROR(nh->get_logger(), "message conversion error");
  }
}

void ROSEndpoint::ros_recv_message(const mavros_msgs::msg::Mavlink::SharedPtr rmsg)
{
  rcpputils::assert_true(!!rmsg, "rmsg not nullptr");

  mavlink::mavlink_message_t mmsg;

  auto ok = mavros_msgs::mavlink::convert(*rmsg, mmsg);
  auto framing = static_cast<Framing>(rmsg->framing_status);

  if (ok) {
    recv_message(&mmsg, framing);
  } else if (auto & nh = this->router) {
    RCLCPP_ERROR(nh->get_logger(), "message conversion error");
  }
}

void ROSEndpoint::diag_run(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // TODO(vooon): make some diagnostics

  stat.addf("Remotes count", "%zu", this->remote_addrs.size());
  size_t idx = 0;
  for (auto addr : this->remote_addrs) {
    stat.addf(utils::format("Remote [%d]", idx++), "%d.%d", addr >> 8, addr & 0xff);
  }

  if (this->is_open()) {
    stat.summary(0, "ok");
  } else {
    stat.summary(2, "closed");
  }
}


#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(mavros::router::Router)
