# MAVLink message: CAMERA_TRACKING_IMAGE_STATUS
# https://mavlink.io/en/messages/common.html#CAMERA_TRACKING_IMAGE_STATUS

std_msgs/Header header

uint8 CAMERA_TRACKING_STATUS_FLAGS_IDLE = 0
uint8 CAMERA_TRACKING_STATUS_FLAGS_ACTIVE = 1
uint8 CAMERA_TRACKING_STATUS_FLAGS_ERROR = 2
uint8 CAMERA_TRACKING_MODE_NONE = 0
uint8 CAMERA_TRACKING_MODE_POINT = 1
uint8 CAMERA_TRACKING_MODE_RECTANGLE = 2
uint8 CAMERA_TRACKING_TARGET_DATA_NONE = 0
uint8 CAMERA_TRACKING_TARGET_DATA_EMBEDDED = 1
uint8 CAMERA_TRACKING_TARGET_DATA_RENDERED = 2
uint8 CAMERA_TRACKING_TARGET_DATA_IN_STATUS = 4

uint8 tracking_status
uint8 tracking_mode
uint8 target_data
float32 point_x
float32 point_y
float32 radius
float32 rec_top_x
float32 rec_top_y
float32 rec_bottom_x
float32 rec_bottom_y
uint8 camera_device_id
