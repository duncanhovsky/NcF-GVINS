# NC-IC extension: optimized online state copied before IC marginalization.
# odom_pose may be globally biased after GNSS degradation; raw_gnss never
# contains the online recovery offset and is the unbiased anchor for map.
std_msgs/Header header
float64 gps_time
uint64 node_id
uint32 revision
int32 segment_id
bool is_keyframe
geometry_msgs/Pose odom_pose
geometry_msgs/Vector3 antenna_lever
bool has_raw_gnss
bool map_only_anchor
bool horizontal_valid
bool vertical_valid
int32 horizontal_health_state
int32 vertical_health_state
geometry_msgs/Point raw_gnss
float64[3] gnss_std
geometry_msgs/Vector3 online_offset
float64 online_yaw
bool online_yaw_observable
float64 estimated_height_bias
bool height_bias_valid
int32 health_state
