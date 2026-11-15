"""Angle-aware scan sectors; distinguish missing data from open-space +infinity."""
import math
import numpy as np


def predictive_velocity(scan,scan_pose,pose,v,w,horizon=.45,clearance=.20):
    """Brake translation before a bounded steering ramp would graze a wall."""
    if v<=0 or scan_pose is None:return v
    ranges=np.asarray(scan.ranges)
    good=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<scan.range_max-.01)
    if not good.any():return v
    angles=scan.angle_min+np.flatnonzero(good)*scan.angle_increment
    lx=-.032+ranges[good]*np.cos(angles);ly=ranges[good]*np.sin(angles)
    sx,sy,sh=scan_pose;x,y,h=pose;c,s=math.cos(sh),math.sin(sh)
    wx=sx+c*lx-s*ly;wy=sy+s*lx+c*ly;c,s=math.cos(h),math.sin(h)
    cloud=np.column_stack((c*(wx-x)+s*(wy-y),-s*(wx-x)+c*(wy-y)))
    current=float(np.min(np.linalg.norm(cloud,axis=1)))
    if current<clearance:v=min(v,.07)
    times=np.linspace(horizon/10,horizon,10)
    for factor in [1.,.75,.5,.25,0.]:
        speed=v*factor
        if abs(w)<1e-8:points=np.column_stack((speed*times,np.zeros_like(times)))
        else:points=np.column_stack((speed/w*np.sin(w*times),speed/w*(1-np.cos(w*times))))
        gaps=np.sqrt(np.min(np.sum((points[:,None,:]-cloud[None,:,:])**2,axis=2),axis=1))
        if gaps.min()>=clearance:return speed
        if current>=.15 and np.all(gaps>=current-.002) and gaps[-1]>current+.005:return speed
    return 0.


def sector(scan, lo, hi):
    values = []
    for i, r in enumerate(scan.ranges):
        a = (math.degrees(scan.angle_min+i*scan.angle_increment)+180)%360-180
        if not lo <= a <= hi:
            continue
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
            values.append(r)
        elif r == math.inf:
            values.append(scan.range_max)
    return min(values) if values else None


def command(front, left, ray45, ray90, near_left, algorithm='improved', settings=None):
    """Return v,w,state; no map lookup or Gazebo ground truth in this function."""
    if None in (front,left,ray45,ray90,near_left):
        return 0.,0.,'INVALID_SCAN'
    if algorithm == 'handout':
        if front < .35: return 0.,-.6,'AVOID_RIGHT'
        if left > .5: return .15,.4,'ACQUIRE_LEFT'
        if left < .3: return .15,-.3,'CLEAR_LEFT'
        return .15,0.,'FOLLOW_LEFT'
    cfg=settings or {}
    wall=cfg.get('wall_distance',.60)
    if algorithm=='flow':
        max_v=cfg.get('sprint_max_v',.5);max_w=cfg.get('flow_max_w',1.4)
        stop=cfg.get('flow_front_stop',.32)
        if front<stop:return 0.,-max_w,'AVOID_RIGHT'
        if near_left<cfg.get('flow_near_left',.22):
            return cfg.get('flow_clear_v',.07),cfg.get('flow_clear_w',-.85),'LEFT_CLEARANCE'
        if ray90>cfg.get('acquire_distance',1.15):
            return cfg.get('flow_acquire_v',.12),cfg.get('flow_acquire_w',.65),'ACQUIRE_LEFT'
        alpha=math.atan2(ray45*math.cos(math.pi/4)-ray90,ray45*math.sin(math.pi/4))
        predicted=ray90*math.cos(alpha)+cfg.get('flow_preview',.25)*math.sin(alpha)
        raw=cfg.get('flow_dist_gain',1.8)*(predicted-wall)+cfg.get('flow_angle_gain',2.0)*alpha
        slow=cfg.get('flow_slow_distance',.95)
        free=max(0.,min(1.,(front-stop)/(slow-stop)))
        side=max(0.,min(1.,(near_left-.22)/.16))
        v=min(max_v/(1+cfg.get('flow_curve_slowdown',2.3)*abs(raw)),.07+(max_v-.07)*free,.12+(max_v-.12)*side)
        w=raw*max(1.,v/cfg.get('flow_speed_gain_reference',.22))
        return v,max(-max_w,min(max_w,w)),'FOLLOW_LEFT'
    if algorithm in ('fast','sprint'):
        # Keep the same wall-following rule and obstacle stop distance. Speed
        # changes continuously with free space instead of a long 0.10 m/s zone.
        stop=cfg.get('front_stop',.40)
        sprint=algorithm=='sprint'
        profile='sprint' if sprint else 'fast'
        max_v=cfg.get(profile+'_max_v',.50 if sprint else .22)
        max_w=cfg.get(profile+'_max_w',1.40 if sprint else 1.10)
        if front < stop: return 0.,-max_w,'AVOID_RIGHT'
        if near_left < cfg.get('near_left',.24):
            return min(max_v,.07),-.85,'LEFT_CLEARANCE'
        if ray90 > cfg.get('acquire_distance',1.7):
            return min(max_v,.12),.65,'ACQUIRE_LEFT'
        alpha=math.atan2(ray45*math.cos(math.pi/4)-ray90,ray45*math.sin(math.pi/4))
        predicted=ray90*math.cos(alpha)+.25*math.sin(alpha)
        raw_w=1.8*(predicted-wall)+2.0*alpha
        w=max(-max_w,min(max_w,raw_w))
        slow=cfg.get(profile+'_slow_distance',1.05 if sprint else .65)
        free=max(0.,min(1.,(front-stop)/max(.05,slow-stop)))
        v=min(max_v/(1.+.35*abs(w)),.07+(max_v-.07)*free)
        if sprint:
            # Higher speed is available only on aligned, open straight sections.
            # Limit turn radius and reduce speed before approaching the left wall.
            v=min(max_v/(1.+3.*abs(raw_w)),.07+(max_v-.07)*free)
            side_free=max(0.,min(1.,(near_left-.22)/.16))
            v=min(v,.12+(max_v-.12)*side_free)
            w=max(-max_w,min(max_w,raw_w*max(1.,v/.22)))
        return v,w,'FOLLOW_LEFT'
    if front < cfg.get('front_stop',.40): return 0.,-.65,'AVOID_RIGHT'
    if near_left < cfg.get('near_left',.24): return .04,-.6,'LEFT_CLEARANCE'
    if ray90 > cfg.get('acquire_distance',1.7): return .10,.5,'ACQUIRE_LEFT'
    alpha = math.atan2(ray45*math.cos(math.pi/4)-ray90,ray45*math.sin(math.pi/4))
    predicted = ray90*math.cos(alpha)+.3*math.sin(alpha)
    w = max(-.85,min(.85,1.5*(predicted-wall)+1.6*alpha))
    v = cfg.get('max_v',.22)/(1+.9*abs(w))
    if front < .85: v = min(v,.10)
    return v,w,'FOLLOW_LEFT'
