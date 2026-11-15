"""Known SDF collision boxes -> conservative grid -> clearance-aware A*.

Coordinates are metres in the world/map frame; array indexing is [row_y, col_x].
No ROS or Gazebo state subscriptions are required for planning.
"""
import hashlib
import heapq
import json
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def pose(element):
    p=list(map(float,element.findtext('pose','0 0 0 0 0 0').split()))
    if abs(p[3])+abs(p[4])>1e-8:
        raise ValueError('Only upright planar wall boxes are supported')
    return p[0],p[1],p[5]


def compose(a,b):
    x,y,h=a; u,v,k=b
    return x+math.cos(h)*u-math.sin(h)*v,y+math.sin(h)*u+math.cos(h)*v,h+k


def load_boxes(world_file):
    boxes=[]
    for model in ET.parse(world_file).findall('./world/model'):
        if model.findtext('static','false').lower()!='true':
            raise ValueError('Planning world must contain static wall models')
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                size=collision.findtext('geometry/box/size')
                if size is None: raise ValueError('Unsupported wall collision geometry')
                sx,sy,_=map(float,size.split())
                x,y,h=compose(compose(pose(model),pose(link)),pose(collision))
                boxes.append((x,y,h,sx,sy))
    if not boxes: raise ValueError('World contains no box obstacles')
    return boxes


class GridMap:
    def __init__(self,world_file,config):
        cfg=config['planning']; self.res=float(cfg.get('resolution',.04))
        self.radius=float(cfg.get('robot_radius',.13))
        self.margin=float(cfg.get('safety_margin',.11))
        if min(self.res,self.radius)<=0 or self.margin<0:
            raise ValueError('Invalid grid resolution or robot clearance')
        self.origin=tuple(cfg.get('bounds',[-6,-6,6,6])[:2])
        xmax,ymax=cfg.get('bounds',[-6,-6,6,6])[2:]
        self.width=math.ceil((xmax-self.origin[0])/self.res)
        self.height=math.ceil((ymax-self.origin[1])/self.res)
        yy,xx=np.mgrid[:self.height,:self.width]
        xx=self.origin[0]+(xx+.5)*self.res
        yy=self.origin[1]+(yy+.5)*self.res
        self.boxes=load_boxes(world_file)
        self.distance=np.full(xx.shape,math.inf)
        for x,y,h,sx,sy in self.boxes:
            dx,dy=xx-x,yy-y; c,s=math.cos(h),math.sin(h)
            qx=np.abs(c*dx+s*dy)-sx/2
            qy=np.abs(-s*dx+c*dy)-sy/2
            distance=np.hypot(np.maximum(qx,0),np.maximum(qy,0))+np.minimum(np.maximum(qx,qy),0)
            self.distance=np.minimum(self.distance,distance)
        half_diagonal=self.res/math.sqrt(2)
        self.occupied=self.distance<=half_diagonal
        self.blocked=self.distance<=self.radius+self.margin+half_diagonal
        # Explicit navigable domain prevents a false route around the outside.
        lo_x,lo_y,hi_x,hi_y=cfg['interior']
        inside=(xx>=lo_x)&(xx<=hi_x)&(yy>=lo_y)&(yy<=hi_y)
        gate=float(cfg.get('gate_half_width',.18))
        sx,sy=config['start']; gx,gy=config['goal']
        entry=(xx>=sx-.12)&(xx<=lo_x+.45)&(np.abs(yy-sy)<=gate)
        exit_=(xx>=hi_x-.45)&(xx<=gx+.15)&(np.abs(yy-gy)<=gate)
        self.domain=inside|entry|exit_
        self.blocked|=~self.domain
        self.world_hash=hashlib.sha256(Path(world_file).read_bytes()).hexdigest()

    def cell(self,point):
        x,y=point
        return math.floor((y-self.origin[1])/self.res),math.floor((x-self.origin[0])/self.res)

    def xy(self,cell):
        r,c=cell
        return self.origin[0]+(c+.5)*self.res,self.origin[1]+(r+.5)*self.res

    def free(self,point):
        r,c=self.cell(point)
        return 0<=r<self.height and 0<=c<self.width and not self.blocked[r,c]

    def line_free(self,a,b,mask=None):
        # Subcell sampling plus both side cells at diagonal crossings forms a
        # conservative supercover; shortcuts cannot cut a blocked grid corner.
        mask=self.blocked if mask is None else mask
        n=max(1,math.ceil(math.dist(a,b)/(self.res*.2)))
        previous=None
        for i in range(n+1):
            t=i/n; cell=self.cell((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])))
            r,c=cell
            if not (0<=r<self.height and 0<=c<self.width) or mask[r,c]: return False
            if previous is not None and r!=previous[0] and c!=previous[1]:
                if mask[r,previous[1]] or mask[previous[0],c]: return False
            previous=cell
        return True

    def search(self,start,goal,record_trace=False):
        if not self.free(start) or not self.free(goal):
            raise ValueError('Start or goal is occupied, inflated, or outside the permitted gates')
        source,target=self.cell(start),self.cell(goal)
        costs={source:0.}; parents={}; closed=set()
        def heuristic(p):
            dr,dc=abs(p[0]-target[0]),abs(p[1]-target[1])
            return self.res*(max(dr,dc)+(math.sqrt(2)-1)*min(dr,dc))
        heap=[(heuristic(source),0.,source)]
        trace=[] if record_trace else None
        while heap:
            _,cost,here=heapq.heappop(heap)
            if here in closed or cost>costs[here]+1e-10: continue
            closed.add(here)
            updates=[] if record_trace else None
            parent=parents.get(here)
            parent_index=-1 if parent is None else parent[0]*self.width+parent[1]
            if here==target:
                if record_trace: trace.append([here[0]*self.width+here[1],cost,heuristic(here),parent_index,[]])
                break
            for dr,dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                r,c=here[0]+dr,here[1]+dc; nxt=(r,c)
                if not (0<=r<self.height and 0<=c<self.width) or self.blocked[r,c]: continue
                if dr and dc and (self.blocked[r,here[1]] or self.blocked[here[0],c]): continue
                penalty=1.+.12/max(self.distance[r,c]-self.radius,.04)
                candidate=cost+self.res*math.hypot(dr,dc)*penalty
                if candidate<costs.get(nxt,math.inf):
                    costs[nxt]=candidate; parents[nxt]=here
                    heapq.heappush(heap,(candidate+heuristic(nxt),candidate,nxt))
                    if record_trace: updates.append([r*self.width+c,candidate,here[0]*self.width+here[1]])
            if record_trace: trace.append([here[0]*self.width+here[1],cost,heuristic(here),parent_index,updates])
        else: raise ValueError('No traversable A* path at the requested robot clearance')
        cells=[target]
        while cells[-1]!=source: cells.append(parents[cells[-1]])
        raw=[tuple(start)]+[self.xy(p) for p in reversed(cells)]+[tuple(goal)]
        # Remove only segments whose entire grid supercover is collision-free.
        route=[raw[0]]; i=0
        while i<len(raw)-1:
            j=i+1
            while j+1<len(raw) and self.line_free(raw[i],raw[j+1]): j+=1
            if not self.line_free(raw[i],raw[j]): raise ValueError('Unsafe endpoint connection')
            if math.dist(route[-1],raw[j])>1e-9: route.append(raw[j])
            i=j
        dense=[route[0]]
        for a,b in zip(route,route[1:]):
            count=max(1,math.ceil(math.dist(a,b)/.06))
            dense.extend((a[0]+(b[0]-a[0])*k/count,a[1]+(b[1]-a[1])*k/count) for k in range(1,count+1))
        result=dict(path=dense,corners=route,grid_path=raw,expanded_nodes=len(closed),weighted_cost=costs[target])
        if record_trace: result['search_trace']=trace
        return result

    def plan(self,start,goal,output,record_trace=False):
        begin=time.perf_counter(); result=self.search(start,goal,record_trace=record_trace)
        trace=result.pop('search_trace',None)
        result.update(start=start,goal=goal,resolution=self.res,robot_radius=self.radius,
                      safety_margin=self.margin,wall_boxes=len(self.boxes),world_sha256=self.world_hash,
                      path_length_m=sum(math.dist(a,b) for a,b in zip(result['path'],result['path'][1:])),
                      planning_seconds=time.perf_counter()-begin,
                      domain='Interior plus designated entrance and exit; outside detours forbidden',
                      source='Known static SDF collision geometry; not SLAM')
        output=Path(output); output.mkdir(parents=True,exist_ok=True)
        (output/'plan.json').write_text(json.dumps(result,indent=2))
        if trace is not None:
            (output/'search_trace.json').write_text(json.dumps(trace,separators=(',',':')))
            result['search_trace']=trace
        for name,mask in [('map',self.occupied),('planning_grid',self.blocked)]:
            pixels=np.flipud(np.where(mask,0,254).astype(np.uint8))
            (output/(name+'.pgm')).write_bytes(f'P5\n{self.width} {self.height}\n255\n'.encode()+pixels.tobytes())
            (output/(name+'.yaml')).write_text(f'image: {name}.pgm\nresolution: {self.res}\norigin: [{self.origin[0]}, {self.origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        return result
