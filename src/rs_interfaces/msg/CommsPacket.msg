# Пакет радиосвязи (Фаза 6): всё, что роботы передают друг другу, идёт через comms_sim в такой обёртке.
# Агент публикует его в /comms/tx, comms_sim доставляет в /<узел>/comms/rx тем, до кого дошло.
builtin_interfaces/Time stamp   # время отправки (сим.)
string src                      # кто отправил: ground_1, drone_2, base
string channel                  # bid, air_bid, pose, beacon (управление) или map, obs (объёмные данные)
uint32 seq                      # номер пакета у отправителя
uint8[] payload                 # сериализованное ROS-сообщение (rclpy.serialization.serialize_message)
