# Объединённая отметка "возможный человек"
uint8 CANDIDATE=0
uint8 CONFIRMED=1
uint8 REJECTED=2
string id
uint8 status
geometry_msgs/Point position
float32 confidence
uint32 n_obs
string[] robots
float32 min_range_m
float32 mean_temp_k
float32 height_m
float32 length_m
builtin_interfaces/Time first_seen
builtin_interfaces/Time status_time
# число RGB-подтверждений (YOLO) и лучшая уверенность
uint32 rgb_hits
float32 rgb_score_max
