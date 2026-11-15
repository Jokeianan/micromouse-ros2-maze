"""Sensor-map flood fill. Unknown space guides search, never execution.

No world/SDF/maze layout is loaded. BFS computes equal-step four-neighbour
distance fields. All executed paths lie inside the observed inflated free map.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_dilation


def flood(blocked, seed):
    """Goal is zero; every reachable neighbour gets current+1 (queue BFS)."""
    h,w=blocked.shape;r,c=seed
    if not (0<r<h-1 and 0<c<w-1) or blocked[r,c]:return np.full((h,w),-1,np.int32)
    wall=blocked.ravel().tolist();values=[-1]*(h*w);source=r*w+c
    values[source]=0;q=deque([source])
    # Every boundary cell is blocked, so a popped cell cannot wrap a row.
    while q:
        here=q.popleft();step=values[here]+1
        for nxt in (here-w,here+1,here+w,here-1):
            if not wall[nxt] and values[nxt]<0:
                values[nxt]=step;q.append(nxt)
    return np.asarray(values,dtype=np.int32).reshape(h,w)


@dataclass
class FloodPlan:
    grid: object
    path: list
    target: object
    kind: str
    field: object
    known_cells: int
    candidates: int


class LiveGrid:
    def __init__(self,msg,start,goal,res=.08,observation=None,minimum_forward=-.16):
        self.res=res;self.start=start;self.goal=goal
        self.origin=(math.floor((min(start[0],goal[0])-2.)/res)*res,
                     math.floor((min(start[1],goal[1])-2.)/res)*res)
        self.width=math.ceil((max(start[0],goal[0])+2.-self.origin[0])/res)
        self.height=math.ceil((max(start[1],goal[1])+2.-self.origin[1])/res)
        a=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
        rr,cc=np.indices(a.shape);mr=msg.info.resolution
        x=msg.info.origin.position.x+(cc+.5)*mr
        y=msg.info.origin.position.y+(rr+.5)*mr
        cols=np.floor((x-self.origin[0])/res).astype(int)
        rows=np.floor((y-self.origin[1])/res).astype(int)
        inside=(rows>=0)&(rows<self.height)&(cols>=0)&(cols<self.width)
        index=(rows*self.width+cols)[inside]
        free_counts=np.bincount(index[(a[inside]>=0)&(a[inside]<20)],minlength=self.width*self.height)
        occ_counts=np.bincount(index[a[inside]>=65],minlength=self.width*self.height)
        self.occupied=occ_counts.reshape(self.height,self.width)>0
        # Conservative aggregation: every 4 cm sample must be free in an 8 cm cell.
        needed=max(1,round((res/mr)**2))
        self.known=(free_counts.reshape(self.height,self.width)>=needed)&~self.occupied
        if observation is not None:
            # SLAM Toolbox does not mark every +inf beam as free. Fuse the live
            # scan into this local planning view; +inf means clear to range_max.
            sx,sy,sh,angle_min,angle_step,range_min,range_max,ranges=observation
            ranges=np.asarray(ranges);valid=((np.isfinite(ranges)&(ranges>=range_min))|np.isposinf(ranges))
            ray=np.minimum(ranges[valid],range_max)
            angles=sh+angle_min+np.flatnonzero(valid)*angle_step
            hit=ray<range_max-.01
            distance=np.arange(0.,range_max,self.res*.35)
            px=sx+np.cos(angles)[:,None]*distance;py=sy+np.sin(angles)[:,None]*distance
            rr=np.floor((py-self.origin[1])/self.res).astype(int);cc=np.floor((px-self.origin[0])/self.res).astype(int)
            good=(distance[None,:]<(ray[:,None]-.6*self.res))&(rr>=0)&(rr<self.height)&(cc>=0)&(cc<self.width)
            self.known[rr[good],cc[good]]=True
            er=np.floor((sy+ray[hit]*np.sin(angles[hit])-self.origin[1])/self.res).astype(int)
            ec=np.floor((sx+ray[hit]*np.cos(angles[hit])-self.origin[0])/self.res).astype(int)
            good=(er>=0)&(er<self.height)&(ec>=0)&(ec<self.width)
            self.occupied[er[good],ec[good]]=True
            self.known&=~self.occupied
        self.clearance=distance_transform_edt(~self.occupied)*res
        self.known_clearance=distance_transform_edt(self.known)*res
        half=res/math.sqrt(2)
        self.blocked=self.clearance<(.13+.08+half)
        self.safe=self.known&~self.blocked&(self.known_clearance>=(.13+half))
        self.track=self.known&(self.clearance>=(.13+.035+half))&(self.known_clearance>=(.13+half))
        # The known entrance is behind the initial pose. Do not explore back out.
        yy,xx=np.indices(self.safe.shape);xx=self.origin[0]+(xx+.5)*res;yy=self.origin[1]+(yy+.5)*res
        hx,hy=math.cos(start[2]),math.sin(start[2])
        allowed=(xx-start[0])*hx+(yy-start[1])*hy>=minimum_forward
        self.blocked|=~allowed;self.safe&=allowed;self.track&=allowed
        for mask in (self.blocked,):mask[[0,-1],:]=True;mask[:,[0,-1]]=True
        for mask in (self.safe,self.track):mask[[0,-1],:]=False;mask[:,[0,-1]]=False

    def cell(self,p):
        return (int(math.floor((p[1]-self.origin[1])/self.res)),int(math.floor((p[0]-self.origin[0])/self.res)))

    def xy(self,cell):
        r,c=cell;return (self.origin[0]+(c+.5)*self.res,self.origin[1]+(r+.5)*self.res)

    def snap(self,p,mask,limit=.28):
        r,c=self.cell(p)
        n=math.ceil(limit/self.res)
        lo_r,hi_r=max(1,r-n),min(self.height-1,r+n+1)
        lo_c,hi_c=max(1,c-n),min(self.width-1,c+n+1)
        rr,cc=np.where(mask[lo_r:hi_r,lo_c:hi_c]);rr+=lo_r;cc+=lo_c
        if not len(rr):return None
        x=self.origin[0]+(cc+.5)*self.res;y=self.origin[1]+(rr+.5)*self.res
        dist=np.hypot(x-p[0],y-p[1]);i=int(np.argmin(dist))
        return (int(rr[i]),int(cc[i])) if dist[i]<=limit else None

    def line_free(self,a,b,mask=None):
        mask=self.safe if mask is None else mask
        n=max(1,math.ceil(math.dist(a,b)/(.2*self.res)))
        previous=None
        for i in range(n+1):
            t=i/n;r,c=self.cell((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])))
            if not(0<=r<self.height and 0<=c<self.width) or not mask[r,c]:return False
            if previous is not None and r!=previous[0] and c!=previous[1]:
                if not mask[r,previous[1]] or not mask[previous[0],c]:return False
            previous=(r,c)
        return True

    def route(self,distance,target,position):
        cells=[target];direction=None
        while distance[cells[-1]]>0:
            r,c=cells[-1];level=distance[r,c]-1
            choices=[(nr,nc) for nr,nc in [(r-1,c),(r,c+1),(r+1,c),(r,c-1)] if distance[nr,nc]==level]
            if not choices:return []
            def merit(p):
                straight=direction==(p[0]-r,p[1]-c)
                return self.clearance[p]+(.06 if straight else 0)
            nxt=max(choices,key=merit);direction=(nxt[0]-r,nxt[1]-c);cells.append(nxt)
        raw=[tuple(position[:2])]+[self.xy(p) for p in reversed(cells)]
        # A snapped source may be slightly outside conservative planning safety.
        if not self.line_free(raw[0],raw[1],self.track):return []
        corners=[raw[0]];i=0
        while i<len(raw)-1:
            j=i+1
            while j+1<len(raw) and self.line_free(raw[i],raw[j+1]):j+=1
            if not self.line_free(raw[i],raw[j],self.track):return []
            if math.dist(corners[-1],raw[j])>1e-6:corners.append(raw[j])
            i=j
        dense=[corners[0]]
        for a,b in zip(corners,corners[1:]):
            n=max(1,math.ceil(math.dist(a,b)/.06))
            dense.extend((a[0]+(b[0]-a[0])*k/n,a[1]+(b[1]-a[1])*k/n) for k in range(1,n+1))
        return dense


def make_plan(msg,start,goal,pose,previous,visits,observation=None,minimum_forward=-.16):
    g=LiveGrid(msg,start,goal,observation=observation,minimum_forward=minimum_forward)
    goal_cell=g.snap(goal,~g.blocked,.16)
    field=flood(g.blocked,goal_cell) if goal_cell is not None else np.full(g.safe.shape,-1,np.int32)
    source=g.snap(pose,g.safe)
    if source is None:return FloodPlan(g,[],None,'WAIT_FREE_SPACE',field,int(g.known.sum()),0)
    reached=flood(~g.safe,source)
    goal_known=g.snap(goal,g.safe,.10)
    if goal_known is not None and reached[goal_known]>=0:
        path=g.route(reached,goal_known,pose)
        if path and g.line_free(path[-1],goal,g.track):path.append(tuple(goal))
        return FloodPlan(g,path,tuple(goal),'GOAL',field,int(g.known.sum()),1)
    # Genuine observed-free/unknown frontiers, excluding the back of wall pixels.
    observed_unknown=~g.known&~g.occupied
    raw=g.known&binary_dilation(observed_unknown)&(g.clearance>.27)
    distance_frontier=distance_transform_edt(~raw)*g.res if raw.any() else np.full(raw.shape,100.)
    candidates=(reached>=0)&(reached*g.res>.30)&(distance_frontier<=.40)&(field>=0)
    rr,cc=np.where(candidates)
    if not len(rr):
        # Scan-reachable advancement when the first map is too small for a frontier band.
        candidates=(reached>=0)&(reached*g.res>.22)&(field>=0)
        rr,cc=np.where(candidates)
    if not len(rr):return FloodPlan(g,[],None,'WAIT_FRONTIER',field,int(g.known.sum()),0)
    x=g.origin[0]+(cc+.5)*g.res;y=g.origin[1]+(rr+.5)*g.res
    scores=field[rr,cc]*g.res+.45*reached[rr,cc]*g.res-.12*np.minimum(g.clearance[rr,cc],.65)
    for vx,vy in visits[-60:]:
        scores+=3.0*np.exp(-((x-vx)**2+(y-vy)**2)/(.35**2))
    # Maintain a target until reached unless the map made it unsafe.
    if previous is not None and math.dist(pose[:2],previous)>.19:
        old=g.snap(previous,g.safe,.12)
        if old is not None and reached[old]>0 and distance_frontier[old]<.55:
            idx=np.argmin((x-previous[0])**2+(y-previous[1])**2)
            scores[idx]-=.7
    order=np.argsort(scores)
    for i in order[:40]:
        target=(int(rr[i]),int(cc[i]));path=g.route(reached,target,pose)
        if len(path)>=2:return FloodPlan(g,path,g.xy(target),'FLOOD_ADVANCE',field,int(g.known.sum()),len(rr))
    return FloodPlan(g,[],None,'WAIT_PATH',field,int(g.known.sum()),len(rr))
