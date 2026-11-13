# Пачка наблюдений одного робота для поиска людей (Фаза 6). seq идёт 1, 2, 3... без пропусков,
# поэтому получатель видит, каких пачек у него нет, и просит их при восстановлении связи.
std_msgs/Header header
string robot
uint32 seq
rs_interfaces/ThermalDetection[] detections
rs_interfaces/RgbDetection[] rgb
float32[] rgb_pose              # x, y, yaw робота на каждую запись rgb (3 * len(rgb))
float64[] view_t                # моменты поз робота (сим. с) для отрицательных наблюдений
float32[] view_pose             # x, y, yaw на каждый момент (3 * len(view_t))
