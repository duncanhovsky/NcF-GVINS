# NC-IC extension: explicit segment lifecycle for the asynchronous map graph.
# event_type: 0=DEGRADED_START, 1=RECOVERY_CONFIRMED, 2=SEGMENT_CLOSED,
# 3=GLOBAL_ALIGNED.
std_msgs/Header header
float64 gps_time
int32 segment_id
int32 event_type
bool deviation_valid
geometry_msgs/Vector3 translation
float64 yaw
bool yaw_observable
int32 supporting_samples
