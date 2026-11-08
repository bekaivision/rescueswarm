# Тепловое пятно, замеченное роботом (позиция в кадре header.frame_id = world)
std_msgs/Header header
string robot
geometry_msgs/Point position
float32 range_m
float32 mean_temp_k
float32 max_temp_k
uint32 blob_px
# по точкам лидара на пятне; NaN, если точек нет
float32 height_m
float32 length_m
uint32 n_lidar_pts
