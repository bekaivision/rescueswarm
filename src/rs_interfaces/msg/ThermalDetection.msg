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
# с каким запасом вокруг пятна нашлись точки лидара: 0 ровно по пятну, 1 (1x1 px), 2 (2x4 px); 255 нет точек
uint8 lidar_pad
# ширина и высота пятна, px
uint16 blob_w_px
uint16 blob_h_px
