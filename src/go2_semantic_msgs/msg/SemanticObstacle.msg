# A single semantically-classified obstacle in map frame.
std_msgs/Header header
string class_name          # YOLO/VLM class label (e.g. "chair", "person")
float32 confidence         # detection confidence [0, 1]
geometry_msgs/Point position  # obstacle centre in map frame (x, y, z)
uint8 cost                 # Nav2 raw cost to assign [0-254]
float32 radius             # radius of influence in metres
string label               # STEP_OVER | CAUTION | AVOID | UNKNOWN; empty = use cost as given
int32 track_id             # object_box_fusion track id, -1 if none
float32 size_x             # oriented footprint in map frame (box sx, sy, yaw); 0 = use radius
float32 size_y
float32 yaw
