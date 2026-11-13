import hashlib
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from rs_worlds import generate as G

FILES = ['world.sdf', 'ground_truth.yaml', 'gt_map.pgm', 'gt_map.yaml',
         'coverage_mask.pgm', 'coverage_mask.yaml', 'preview.png']


def sha(d):
    return {f: hashlib.sha256(open(os.path.join(d, f), 'rb').read()).hexdigest() for f in FILES}


def gen(tmp_path, seed, size='medium', name=None):
    out = str(tmp_path / (name or f'{size}_{seed}'))
    G.main(['--seed', str(seed), '--out', out, '--size', size])
    return out


def test_deterministic(tmp_path):
    a = gen(tmp_path, 5, name='a')
    b = gen(tmp_path, 5, name='b')
    assert sha(a) == sha(b)


def test_different_seeds_differ(tmp_path):
    a = sha(gen(tmp_path, 1))
    b = sha(gen(tmp_path, 2))
    assert a['world.sdf'] != b['world.sdf']
    assert a['gt_map.pgm'] != b['gt_map.pgm']


def test_outputs_and_xml(tmp_path):
    out = gen(tmp_path, 3)
    for f in FILES:
        assert os.path.getsize(os.path.join(out, f)) > 0
    root = ET.parse(os.path.join(out, 'world.sdf')).getroot()
    assert root.tag == 'sdf' and root.attrib['version'] == '1.9'
    assert root.find('world').attrib['name'] == 'rescue'
    models = [m.attrib['name'] for m in root.iter('model')]
    assert 'walls' in models and any(m.startswith('person_') for m in models)
    gt = yaml.safe_load(open(os.path.join(out, 'ground_truth.yaml')))
    assert len(gt['spawn_poses']) == 5
    assert gt['robot_radius'] == 0.3


@pytest.mark.parametrize('size', ['small', 'medium'])
@pytest.mark.parametrize('seed', range(1, 21))
def test_invariants(size, seed):
    w = G.build_world(seed, size)
    g, an = w['grid'], w['an']
    gt = G.ground_truth(w)
    # зона старта свободна и достижима (она снаружи здания, поэтому вне маски)
    assert len(w['spawn']) == 5
    for sp in w['spawn']:
        j, i = g.cell(sp['x'], sp['y'])
        assert not w['occ'][j, i]
        assert an['reach'][j, i]
    # люди вне препятствий
    k = 0
    for p in w['people']:
        assert not g.hits(w['wall_occ'] | w['rubble_occ'], p['x'], p['y'],
                          G.PERSON_BODY[0], G.PERSON_BODY[1], p['yaw'])
        k += 1
    lo, hi = G.SIZES[size]['people']
    assert lo <= len(w['people']) <= hi
    assert len(gt['people']) == len(w['people'])
    # достижимая площадь
    assert w['int_reach'] >= 0.7 * w['int_free']
    types = [z['type'] for z in gt['zones']]
    assert types.count('narrow_passage') >= 2
    assert types.count('dead_end') >= 2
    # узкие проходы проходимы для диска робота
    for z in gt['zones']:
        if z['type'] == 'narrow_passage':
            assert an['eroded'][g.rect_slice(z['rect'])].any()
    lo, hi = G.SIZES[size]['rubble']
    assert lo <= len(w['rubble']) <= hi
    assert gt['stats']['mask_area_m2'] > 20
    # проёмы внешней стены рядом со стартом
    assert any(d['kind'] == 'entrance' for d in w['lay']['doors'])


def test_gt_map_matches_obstacles(tmp_path):
    out = gen(tmp_path, 4)
    data = open(os.path.join(out, 'gt_map.pgm'), 'rb').read()
    head, _, rest = data.partition(b'255\n')
    wd, h = map(int, head.split()[1:3])
    arr = np.frombuffer(rest, dtype=np.uint8).reshape(h, wd)
    assert set(np.unique(arr)) <= {0, 254}
    assert (arr == 0).sum() > 0


@pytest.mark.parametrize('seed', range(1, 21))
def test_decoys(seed):
    w = G.build_world(seed, 'medium')
    gt = G.ground_truth(w)
    assert 3 <= len(gt['decoys']) <= 6
    assert any(d['type'] == 'warm_debris' for d in gt['decoys'])
    for d in gt['decoys']:
        for p in gt['people']:
            assert ((d['x'] - p['x']) ** 2 + (d['y'] - p['y']) ** 2) ** 0.5 >= G.PEOPLE_DECOY_GAP


@pytest.mark.parametrize('size', ['small', 'medium'])
@pytest.mark.parametrize('seed', range(1, 21))
def test_air_spawns(size, seed):
    """Стартовые позы дронов: >= 2, южнее наземных, на свободной земле, курс как у наземных."""
    w = G.build_world(seed, size)
    g = w['grid']
    gt = G.ground_truth(w)
    air = gt['air_spawns']
    assert len(air) >= 2
    ground_y = gt['spawn_poses'][0]['y']
    xs = w['lay']['xs']
    for a in air:
        assert a['y'] == pytest.approx(ground_y - 2.0, abs=1e-3)
        assert abs(a['x'] - xs) <= 3.0 + 1e-3
        assert a['yaw'] == pytest.approx(gt['spawn_poses'][0]['yaw'])
        j, i = g.cell(a['x'], a['y'])
        assert not w['occ'][j, i]
        assert g.ymin <= a['y'] <= g.ymax
    # позы дронов не совпадают друг с другом
    assert len({(a['x'], a['y']) for a in air}) == len(air)


@pytest.mark.parametrize('size', ['small', 'medium'])
@pytest.mark.parametrize('seed', range(1, 11))
def test_spawns_n5_with_3_drones(size, seed):
    """N=5 наземных + 3 дрона (и N=4 + 3): позы в зоне старта, не пересекаются, не в препятствиях."""
    w = G.build_world(seed, size)
    g = w['grid']
    gt = G.ground_truth(w)
    ground, air = gt['spawn_poses'], gt['air_spawns']
    assert len(ground) >= 5 and len(air) >= 3
    sx0, sy0, sx1, sy1 = next(z for z in gt['zones'] if z['type'] == 'spawn')['rect']
    r = G.ROBOT_RADIUS
    occ = w['wall_occ'] | w['rubble_occ']
    for k, p in enumerate(ground[:5]):
        # центр и круг робота внутри зоны старта
        assert sx0 + r <= p['x'] <= sx1 - r and sy0 + r <= p['y'] <= sy1 - r, k
        assert not g.hits(occ, p['x'], p['y'], 2 * r, 2 * r, 0.0), k
    for a in range(5):
        for b in range(a + 1, 5):
            d = math.hypot(ground[a]['x'] - ground[b]['x'], ground[a]['y'] - ground[b]['y'])
            assert d >= 2 * r + 0.2, (a, b)
    for k, p in enumerate(air[:3]):
        assert not g.hits(occ, p['x'], p['y'], 0.8, 0.8, 0.0), k
        for q in ground[:5]:
            assert math.hypot(p['x'] - q['x'], p['y'] - q['y']) >= 1.0
    for a in range(len(air)):
        for b in range(a + 1, len(air)):
            assert math.hypot(air[a]['x'] - air[b]['x'], air[a]['y'] - air[b]['y']) >= 1.4, (a, b)
