# EKF Status message including flags and variances.
#
# https://mavlink.io/en/messages/ardupilotmega.html#EKF_STATUS_REPORT

std_msgs/Header header

uint16 flags
float32 velocity_variance
float32 pos_horiz_variance
float32 pos_vert_variance
float32 compass_variance
float32 terrain_alt_variance
float32 airspeed_variance
