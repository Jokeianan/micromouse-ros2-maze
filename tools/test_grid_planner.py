"""Geometry and reachability checks independent of ROS/Gazebo."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('grid_planner',ROOT/'src/micromouse_practice/micromouse_practice/grid_planner.py')
m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

def tiny(mask):
    g=m.GridMap.__new__(m.GridMap); g.res=1.; g.origin=(0.,0.)
    g.blocked=np.array(mask,dtype=bool); g.height,g.width=g.blocked.shape
    g.distance=np.full(g.blocked.shape,10.); g.radius=.13
    return g

# A point robot cannot pass diagonally through two touching blocked cells.
g=tiny([[0,1],[1,0]])
assert not g.line_free((.5,.5),(1.5,1.5))
try: g.search((.5,.5),(1.5,1.5))
except ValueError: pass
else: raise AssertionError('A* cut a blocked diagonal corner')

g=tiny([[0,0,0],[0,1,0],[0,0,0]])
p=g.search((.5,.5),(2.5,2.5))
assert all(g.line_free(a,b) for a,b in zip(p['path'],p['path'][1:]))
assert not g.free((-.1,.5)) and not g.free((3.,.5))
try: g.search((.5,.5),(1.5,1.5))
except ValueError: pass
else: raise AssertionError('Occupied goal was accepted')

# Wall geometry composes model, link and collision offsets/yaw.
with tempfile.TemporaryDirectory() as d:
    w=Path(d)/'rotated.world'
    w.write_text('<sdf><world><model name="test"><static>true</static><pose>1 2 0 0 0 1.5707963267948966</pose><link name="l"><pose>1 0 0 0 0 0</pose><collision name="c"><pose>1 0 0 0 0 0</pose><geometry><box><size>2 0.1 1</size></box></geometry></collision></link></model></world></sdf>')
    x,y,h,sx,sy=m.load_boxes(w)[0]
    assert abs(x-1)<1e-8 and abs(y-4)<1e-8 and abs(h-math.pi/2)<1e-8

cfg=json.loads((ROOT/'src/micromouse_practice/config/maps.json').read_text())['migong']
grid=m.GridMap(ROOT/'source_worlds/migong.world',cfg)
plan=grid.search(cfg['start'],cfg['goal'])
assert len(grid.boxes)==140
assert all(grid.line_free(a,b) for a,b in zip(plan['path'],plan['path'][1:]))
assert all(abs(y)<5.08 and (abs(x)<5.05 or (x<0 and abs(y+3.48)<.2) or (x>0 and abs(y-3.48)<.2)) for x,y in plan['path'])
assert not grid.free((-5.4,0)) and not grid.free((5.4,0))
# A real corner-tracking deviation must allow a safe correction, even though
# the more conservative planning layer rejects a direct shortcut from it.
q=(-2.293299718658185,3.2399663278788213); target=(-2.229129224174973,3.28776483987502)
tracking=(grid.distance<=grid.radius+.05+grid.res/math.sqrt(2))|~grid.domain
assert not grid.line_free(q,target) and grid.line_free(q,target,tracking)
print('PASS: rotated collision geometry, occupied goal, no-path failure, diagonal guard, endpoint/grid boundaries, all 140 walls, full planned path, no outside shortcut')
