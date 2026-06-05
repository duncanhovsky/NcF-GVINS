# NC-IC extension: compact relative-motion constraint emitted by the online
# estimator for asynchronous degraded-segment map relocation.
std_msgs/Header header
int32 segment_id
uint64 node_i
uint64 node_j
uint32 revision
float64 time_i
float64 time_j
int32 source_type
geometry_msgs/Vector3 delta_position
float64 delta_yaw
float64[4] std
float64 quality_score
int32 health_state
bool switchable
