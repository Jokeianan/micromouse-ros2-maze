"""Meaningful topology, unknown-space and thin-wall aggregation regressions."""
import numpy as np
from nav_msgs.msg import OccupancyGrid
from micromouse_practice.flood_planner import flood,LiveGrid,make_plan

blocked=np.zeros((9,9),bool);blocked[[0,-1],:]=True;blocked[:,[0,-1]]=True
blocked[1:7,4]=True
d=flood(blocked,(2,6))
assert d[2,6]==0 and d[2,2]>abs(2-2)+abs(2-6)
assert d[3,4]==-1
blocked[7,4]=True
assert flood(blocked,(2,6))[2,2]==-1

m=OccupancyGrid();m.info.width=m.info.height=120;m.info.resolution=.04;m.info.origin.orientation.w=1.
m.info.origin.position.x=m.info.origin.position.y=-2.4
grid=np.full((120,120),-1,np.int8);grid[35:85,35:85]=0;grid[:,80]=100;m.data=grid.ravel().tolist()
g=LiveGrid(m,(-.5,0.,0.),(1.5,0.))
# A one-pixel sensor wall must survive 2:1 coarsening and inflation.
r,c=g.cell((.82,0.));assert g.blocked[r,c]
assert not g.line_free((0.,0.),(1.,0.),g.track)
plan=make_plan(m,(-.5,0.,0.),(1.5,0.),(-.5,0.,0.),None,[])
assert plan.kind!='GOAL'
for a,b in zip(plan.path,plan.path[1:]):assert plan.grid.line_free(a,b,plan.grid.track)
assert g.track.sum()<g.known.sum()
empty=OccupancyGrid();empty.info.width=empty.info.height=100;empty.info.resolution=.04
empty.info.origin.position.x=empty.info.origin.position.y=-2.;empty.info.origin.orientation.w=1.
empty.data=[-1]*10000
scan=(0.,0.,0.,-np.pi,np.pi/180,.12,3.5,[float('inf')]*360)
live=LiveGrid(empty,(0.,0.,0.),(1.,0.),observation=scan)
assert live.track[live.cell((0.,0.))]
assert live.track[live.cell((.5,0.))]
assert not live.occupied.any()
assert live.known.sum()>0
print('Flood topology, unreachable goal, thin wall, unknown-space safety: PASS')
