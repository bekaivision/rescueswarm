# Слой карты одного робота (Фаза 6): только по его собственным датчикам.
# Одна клетка = int8, построчно (индекс = iy * width + ix):
#   -128 клетку источник не видел; 127 клетка не менялась (только в дельте, base_version > 0);
#   иначе q = round(log-odds * 20), от -100 до 100.
std_msgs/Header header          # stamp = время снимка, frame_id = world
string source                   # чей слой: ground_1, drone_2
uint8 KIND_GROUND=0
uint8 KIND_AIR=1
uint8 kind
uint32 version                  # версия слоя у источника
uint32 base_version             # 0 = весь слой; иначе только клетки, изменившиеся после этой версии
uint32 width
uint32 height
float32 resolution
float64 origin_x
float64 origin_y
uint8 ENC_RAW=0
uint8 ENC_ZLIB=1
uint8 encoding
uint8[] data
