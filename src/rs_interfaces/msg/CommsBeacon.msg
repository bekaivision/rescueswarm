# Маяк узла сети (Фаза 6), 1 Гц: что у узла уже есть. Соседи по нему решают, что прислать.
std_msgs/Header header
string node
string[] map_sources
uint32[] map_versions           # до какой версии слой источника у меня без пропусков
string[] obs_sources
uint32[] obs_seqs               # до какого seq пачки наблюдений источника у меня без пропусков
string[] neighbors              # кого я слышал за последние несколько секунд
