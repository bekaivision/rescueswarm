import xml.etree.ElementTree as ET

import pytest

from rs_worlds.generate import build_world, world_sdf
from rs_worlds.px4_world import add_px4_systems


def _world():
    return world_sdf(build_world(1, "medium"))


def test_adds_systems_and_geo_and_stays_valid_xml():
    s = add_px4_systems(_world())
    root = ET.fromstring(s.split('?>', 1)[1])
    w = root.find('world')
    names = [p.get('name') for p in w.findall('plugin')]
    for n in ('gz::sim::systems::Physics', 'gz::sim::systems::Imu', 'gz::sim::systems::Magnetometer',
              'gz::sim::systems::AirPressure', 'gz::sim::systems::NavSat'):
        assert n in names
    sc = w.find('spherical_coordinates')
    assert sc is not None and sc.find('world_frame_orientation').text == 'ENU'
    assert w.find('magnetic_field') is not None
    # шаг физики не меняется
    assert w.find('physics/max_step_size').text == '0.002'


def test_idempotent():
    s = add_px4_systems(_world())
    assert add_px4_systems(s) == s


def test_no_physics_plugin_is_error():
    with pytest.raises(ValueError):
        add_px4_systems('<sdf><world name="x"></world></sdf>')
