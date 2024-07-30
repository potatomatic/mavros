# Heartbeat.msg
#
# ROS representation of MAVLink HEARTBEAT
# See mavlink message documentation here:
# https://mavlink.io/en/messages/minimal.html#HEARTBEAT

std_msgs/Header header
uint8 type
uint8 autopilot
uint8 base_mode
uint32 custom_mode
uint8 system_status
uint8 mavlink_version
