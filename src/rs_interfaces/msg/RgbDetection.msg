# Человек на RGB-кадре (YOLO). Направление относительно оптической оси камеры робота
std_msgs/Header header
string robot
# угол центра рамки от оси камеры, рад; плюс = влево (против часовой, как yaw)
float32 bearing_rad
float32 half_width_rad
float32 score
# рамка в пикселях: x0 y0 x1 y1
int32[4] box_px
