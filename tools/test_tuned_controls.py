"""Safety/geometry regressions for the tuning and fresh-pose changes."""
import math
from types import SimpleNamespace as S
import numpy as np
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from micromouse_practice.laser import command,predictive_velocity
from sensor_msgs.msg import LaserScan
from micromouse_practice.flood_planner import LiveGrid
from micromouse_practice.flood_navigation import FloodNavigator

cfg=dict(wall_distance=.4,flow_max_w=1.4,flow_clear_v=.07,flow_clear_w=-.85)
v,w,state=command(3.,.4,.4*math.sqrt(2),.4,.45,'flow',cfg)
assert .49<=v<=.5 and abs(w)<1e-7
assert command(.2,.4,.6,.4,.4,'flow',cfg)==(0.,-1.4,'AVOID_RIGHT')
assert command(1.,.4,.6,.4,.18,'flow',cfg)==(.07,-.85,'LEFT_CLEARANCE')
assert command(None,.4,.6,.4,.4,'flow',cfg)==(0.,0.,'INVALID_SCAN')
scan=LaserScan();scan.angle_min=0.;scan.angle_increment=math.pi/180
scan.range_min=.12;scan.range_max=3.5;scan.ranges=[float('inf')]*360
scan.ranges[0]=.30
assert predictive_velocity(scan,(0.,0.,0.),(0.,0.,0.),.5,0.)<.5
for angle in range(15,166):scan.ranges[angle]=.18/math.sin(math.radians(angle))
scan.ranges[0]=float('inf')
assert predictive_velocity(scan,(0.,0.,0.),(0.,0.,0.),.1,-.8)>0.

node=object.__new__(FloodNavigator);calls=[]
def tf(x,y,h):return S(transform=S(translation=S(x=x,y=y),rotation=S(x=0.,y=0.,z=math.sin(h/2),w=math.cos(h/2))))
def lookup(parent,child,stamp):
    calls.append((parent,child,stamp.nanoseconds))
    return tf(1.,2.,math.pi/2) if parent=='map' else tf(2.,3.,.1)
node.buffer=S(lookup_transform=lookup)
pose=node.frame_pose('base_scan',Time(seconds=42))
assert np.allclose(pose,[-2.,4.,math.pi/2+.1])
assert calls[0]==('map','odom',0) and calls[1]==('odom','base_scan',42000000000)

m=OccupancyGrid();m.info.width=m.info.height=100;m.info.resolution=.04
m.info.origin.position.x=m.info.origin.position.y=-2.;m.info.origin.orientation.w=1.;m.data=[-1]*10000
observation=(.8,0.,0.,-math.pi,math.pi/180,.12,3.5,[float('inf')]*360)
g=LiveGrid(m,(0.,0.,0.),(1.,0.),observation=observation,minimum_forward=.40)
assert not g.track[g.cell((0.,0.))] and g.track[g.cell((.8,0.))]
print('Wall obstacle priorities, fresh odometry composition, and entrance return guard: PASS')
