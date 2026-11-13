"""Мир для дронов PX4 (Фаза 5): добавляет в готовый world.sdf то, без чего PX4 SITL не взлетит.

PX4 читает из Gazebo IMU, магнитометр, барометр и GPS (navsat) своей модели. Для этого в мире нужны
системы Magnetometer, AirPressure, NavSat (Imu уже есть), географическая привязка
<spherical_coordinates> (ENU: x на восток, y на север) и магнитное поле. Значения как в мире PX4
default.sdf (v1.17.0). Шаг физики не трогаем: проверка px4_smoke показала, что PX4 летает и на 2 мс.
Для мира без дронов PX4 функция не вызывается, мир остаётся прежним байт в байт.
"""
SYSTEMS = (
    '<plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>',
    '<plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>',
    '<plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>',
)
GEO = '''    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
'''
PHYSICS_PLUGIN = 'name="gz::sim::systems::Physics"/>'


def add_px4_systems(sdf):
    """Текст world.sdf -> текст с системами и привязкой для PX4. Повторный вызов ничего не меняет."""
    if 'gz::sim::systems::NavSat' in sdf:
        return sdf
    i = sdf.find(PHYSICS_PLUGIN)
    if i < 0:
        raise ValueError('в мире нет системы Physics: не понимаю, куда вставлять системы PX4')
    i += len(PHYSICS_PLUGIN)
    ins = ''.join('\n    ' + s for s in SYSTEMS)
    sdf = sdf[:i] + ins + sdf[i:]
    j = sdf.rfind('</world>')
    if j < 0:
        raise ValueError('нет </world>')
    if '<spherical_coordinates>' not in sdf:
        sdf = sdf[:j] + GEO + '  ' + sdf[j:]
    return sdf


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description='добавить в world.sdf системы и привязку для PX4 (на месте)')
    ap.add_argument('world_sdf')
    a = ap.parse_args(argv)
    with open(a.world_sdf) as f:
        s = f.read()
    with open(a.world_sdf, 'w') as f:
        f.write(add_px4_systems(s))


if __name__ == '__main__':
    main()
