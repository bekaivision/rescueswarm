import hashlib
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
                          G.PERSON_SIZE[0], G.PERSON_SIZE[1], p['yaw'])
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
