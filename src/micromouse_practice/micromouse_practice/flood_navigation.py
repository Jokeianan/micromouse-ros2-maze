"""Live SLAM + goal-directed flood fill + acceleration-limited local control."""
from concurrent.futures import ThreadPoolExecutor
import csv,json,math,signal,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock,ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from tf2_ros import Buffer,TransformListener
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import OccupancyGrid,Odometry,Path as RosPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from maze_control.controller_base import latched_qos
from maze_control.control_math import Polyline,wrap
from .flood_planner import make_plan
from .laser import sector


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class FloodNavigator(Node):
    def __init__(self):
        super().__init__('tb3_flood_navigation')
        self.output=Path(self.declare_parameter('output_dir','/tmp/flood').value).expanduser();self.output.mkdir(parents=True,exist_ok=True)
        self.max_v=float(self.declare_parameter('max_v',.5).value)
        self.timeout=float(self.declare_parameter('timeout',1800.).value)
        self.delta=(float(self.declare_parameter('goal_dx',10.97).value),float(self.declare_parameter('goal_dy',6.96).value))
        self.entry_distance=float(self.declare_parameter('entry_distance',.8).value)
        self.entry_complete=False
        self.lookahead=float(self.declare_parameter('lookahead',.34).value)
        self.curve_slowdown=float(self.declare_parameter('curvature_slowdown',1.1).value)
        self.align_threshold=float(self.declare_parameter('align_threshold',.65).value)
        self.accel=float(self.declare_parameter('linear_accel',.4).value)
        self.decel=float(self.declare_parameter('linear_decel',.7).value)
        self.angular_accel=float(self.declare_parameter('angular_accel',2.5).value)
        self.prediction_seconds=float(self.declare_parameter('nominal_prediction_seconds',.72).value)
        self.replan_period=float(self.declare_parameter('replan_period',1.0).value)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.status_pub=self.create_publisher(String,'/control_status',latched_qos())
        self.path_pub=self.create_publisher(RosPath,'/reference_path',latched_qos())
        self.local_pub=self.create_publisher(RosPath,'/local_trajectory',10)
        self.target_pub=self.create_publisher(PoseStamped,'/navigation_target',10)
        self.field_pub=self.create_publisher(OccupancyGrid,'/flood_distance',latched_qos())
        self.create_subscription(OccupancyGrid,'/map',self.map_cb,latched_qos())
        self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data)
        self.map=self.scan=self.start=self.goal=self.pose=None
        self.map_at=self.scan_at=self.odom_at=-math.inf
        self.scans=self.odoms=0;self.started=None;self.done=False
        self.plan=self.path=None;self.target=None;self.progress=0.
        self.v=self.w=0.;self.last_control=None;self.last_state=None
        self.visits=[];self.pool=ThreadPoolExecutor(max_workers=1);self.future=None;self.replans=0
        self.last_motion=None;self.motion_at=None;self.recovery_until=0.
        self.stream=(self.output/'flood_telemetry.csv').open('w',newline='');self.writer=csv.writer(self.stream)
        self.writer.writerow(['t','x','y','yaw','front','scan_min','v','w','state','goal_error','replans','target_x','target_y','progress'])
        self.events=(self.output/'flood_plans.jsonl').open('w')
        self.create_timer(.05,self.control)
        self.create_timer(self.replan_period,self.replan)
        self.create_timer(.1,self.watchdog,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def map_cb(self,m):self.map=m;self.map_at=time.monotonic()
    def scan_cb(self,m):self.scan=m;self.scan_at=time.monotonic();self.scans+=1
    def odom_cb(self,m):self.odom_at=time.monotonic();self.odoms+=1

    def watchdog(self):
        if self.done or time.monotonic()-min(self.scan_at,self.odom_at)>.7 or time.monotonic()-self.map_at>5.:
            self.v=0.;self.w=0.;self.pub.publish(Twist())

    def frame_pose(self,frame,stamp=None):
        # map->odom is a slowly updated SLAM correction. Chained Time(0) would
        # force the robot pose back to that old common timestamp while stopped.
        # Apply its latest correction to fresh (or scan-time) odometry instead.
        try:
            m=self.buffer.lookup_transform('map','odom',rclpy.time.Time()).transform
            o=self.buffer.lookup_transform('odom',frame,stamp or rclpy.time.Time()).transform
            angle=yaw(m.rotation);c,s=math.cos(angle),math.sin(angle)
            return (m.translation.x+c*o.translation.x-s*o.translation.y,
                    m.translation.y+s*o.translation.x+c*o.translation.y,wrap(angle+yaw(o.rotation)))
        except Exception:return None

    def get_pose(self):return self.frame_pose('base_footprint')

    def ros_path(self,points):
        p=RosPath();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg()
        for x,y,*_ in points:
            q=PoseStamped();q.header=p.header;q.pose.position.x=float(x);q.pose.position.y=float(y);q.pose.orientation.w=1.;p.poses.append(q)
        return p

    def consume_plan(self):
        if self.future is not None:
            if not self.future.done():return
            try:
                plan=self.future.result();self.replans+=1
                keep=False
                if self.path is not None and self.target is not None and plan.target is not None and self.plan is not None:
                    same=self.plan.kind==plan.kind and math.dist(self.target,plan.target)<.40
                    if same and math.dist(self.pose[:2],self.target)>.22:
                        forward=[self.path.point_at(self.progress)]+[p for p,s in zip(self.path.points,self.path.lengths) if s>self.progress]
                        keep=all(plan.grid.line_free(a,b,plan.grid.track) for a,b in zip(forward,forward[1:]))
                if keep:
                    plan.path=list(self.path.points);plan.target=self.target
                else:
                    self.path=Polyline(plan.path) if len(plan.path)>1 else None;self.progress=0.
                self.plan=plan;self.target=plan.target
                self.path_pub.publish(self.ros_path(plan.path))
                if plan.target:
                    target=PoseStamped();target.header.frame_id='map';target.header.stamp=self.get_clock().now().to_msg()
                    target.pose.position.x,target.pose.position.y=plan.target;target.pose.orientation.w=1.;self.target_pub.publish(target)
                g=plan.grid;m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
                m.info.resolution=g.res;m.info.width=g.width;m.info.height=g.height
                m.info.origin.position.x,m.info.origin.position.y=g.origin;m.info.origin.orientation.w=1.
                scaled=np.where(plan.field<0,-1,np.minimum(99,plan.field/4)).astype(np.int8)
                m.data=scaled.ravel().tolist();self.field_pub.publish(m)
                record=dict(t=self.elapsed(),kind=plan.kind,target=plan.target,path=plan.path,
                            known_cells=plan.known_cells,frontier_candidates=plan.candidates,replan=self.replans)
                self.events.write(json.dumps(record)+'\n');self.events.flush()
                if self.replans%10==1:
                    np.savez_compressed(self.output/f'flood_{self.replans:04d}.npz',field=plan.field,known=g.known,occupied=g.occupied,
                                        safe=g.safe,path=np.array(plan.path).reshape(-1,2),origin=g.origin,resolution=g.res,goal=self.goal)
            except Exception as exc:
                self.get_logger().error('Planning failed: '+repr(exc));self.path=None
            self.future=None

    def replan(self):
        if self.done or self.map is None or self.start is None or self.pose is None:return
        self.consume_plan()
        if self.future is not None:return
        planning_goal=self.goal
        along=(self.pose[0]-self.start[0])*math.cos(self.start[2])+(self.pose[1]-self.start[1])*math.sin(self.start[2])
        if along>=self.entry_distance-.1:self.entry_complete=True
        if not self.entry_complete:
            planning_goal=(self.start[0]+self.entry_distance*math.cos(self.start[2]),self.start[1]+self.entry_distance*math.sin(self.start[2]))
        observation=None
        sensor=self.frame_pose(self.scan.header.frame_id,rclpy.time.Time.from_msg(self.scan.header.stamp))
        if sensor is not None:
            observation=(*sensor,
                         self.scan.angle_min,self.scan.angle_increment,self.scan.range_min,self.scan.range_max,list(self.scan.ranges))
        minimum_forward=.40 if self.entry_complete else -.16
        self.future=self.pool.submit(make_plan,self.map,self.start,planning_goal,self.pose,self.target,list(self.visits),observation,minimum_forward)

    def elapsed(self):
        return 0. if self.started is None else self.get_clock().now().nanoseconds/1e9-self.started

    def motion(self,pose,v,w):
        x,y,h=pose;points=[]
        for _ in range(12):
            dt=self.prediction_seconds/12
            if abs(w)<1e-8:nx=x+v*dt*math.cos(h);ny=y+v*dt*math.sin(h)
            else:nx=x+v/w*(math.sin(h+w*dt)-math.sin(h));ny=y-v/w*(math.cos(h+w*dt)-math.cos(h))
            x,y,h=nx,ny,h+w*dt;points.append((x,y,h))
        return np.asarray(points)

    def local_command(self,pose,nominal_v,nominal_w,dt):
        g=self.plan.grid
        # A dynamic window enforces acceleration limits, then scores short arcs.
        vlo=max(0.,self.v-self.decel*dt);vhi=min(self.max_v,self.v+self.accel*dt)
        wlo=max(-1.4,self.w-self.angular_accel*dt);whi=min(1.4,self.w+self.angular_accel*dt)
        vv=np.unique(np.append(np.linspace(vlo,vhi,3),np.clip(nominal_v,vlo,vhi)))
        ww=np.unique(np.append(np.linspace(wlo,whi,5),np.clip(nominal_w,wlo,whi)))
        valid=np.isfinite(self.scan.ranges)&(np.asarray(self.scan.ranges)>=self.scan.range_min)&(np.asarray(self.scan.ranges)<self.scan.range_max-.01)
        rays=np.asarray(self.scan.ranges)[valid];angles=(self.scan.angle_min+np.arange(len(self.scan.ranges))*self.scan.angle_increment)[valid]
        sensor=self.frame_pose(self.scan.header.frame_id,rclpy.time.Time.from_msg(self.scan.header.stamp))
        if sensor is None:return 0.,0.,[]
        sx,sy,sh=sensor
        cloud=np.column_stack((sx+rays*np.cos(angles+sh),sy+rays*np.sin(angles+sh)))
        best=None
        for v in vv:
            for w in ww:
                points=self.motion(pose,float(v),float(w))
                rows=np.floor((points[:,1]-g.origin[1])/g.res).astype(int);cols=np.floor((points[:,0]-g.origin[0])/g.res).astype(int)
                if np.any((rows<0)|(cols<0)|(rows>=g.height)|(cols>=g.width)):continue
                if not np.all(g.track[rows,cols]):continue
                if len(cloud):
                    clearance=float(np.sqrt(np.min(np.sum((points[:,:2,None]-cloud.T[None,:,:])**2,axis=1))))
                    if clearance<.165:continue
                else:clearance=1.
                # Desired pursuit arc supplies direction; obstacle cost refines it.
                score=3.*abs(v-nominal_v)+.65*abs(w-nominal_w)+.10*abs(w-self.w)+.005/max(.02,clearance-.13)
                if best is None or score<best[0]:best=(score,float(v),float(w),points)
        if best is None:
            # Safety braking may stop translation immediately, while a feasible
            # in-place arc retains bounded steering instead of resetting omega.
            r,c=g.cell(pose)
            clear=not len(cloud) or float(np.min(np.linalg.norm(cloud-np.asarray(pose[:2]),axis=1)))>=.165
            if 0<=r<g.height and 0<=c<g.width and g.track[r,c] and clear:
                w=float(np.clip(nominal_w,wlo,whi))
                return 0.,w,self.motion(pose,0.,w).tolist()
            return 0.,0.,[]
        return best[1],best[2],best[3].tolist()

    def control(self):
        if self.done:return
        now=self.get_clock().now().nanoseconds/1e9
        if self.last_control is not None and now<=self.last_control+1e-7:return
        dt=.05 if self.last_control is None else min(.15,now-self.last_control);self.last_control=now
        self.consume_plan()
        pose=self.get_pose();self.pose=pose
        if pose is None or self.scan is None or self.map is None or time.monotonic()-min(self.scan_at,self.odom_at)>.7 or time.monotonic()-self.map_at>5.:
            self.v=self.w=0.;self.pub.publish(Twist());return
        if self.start is None:
            self.start=pose;c,s=math.cos(pose[2]),math.sin(pose[2]);dx,dy=self.delta
            self.goal=(pose[0]+c*dx-s*dy,pose[1]+s*dx+c*dy);self.started=now
            self.last_motion=pose[:2];self.motion_at=now
        elapsed=self.elapsed();x,y,h=pose;error=math.dist(pose[:2],self.goal)
        front=sector(self.scan,-18,18);near=sector(self.scan,-180,180)
        v=w=0.;state='WAIT_PLAN';prediction=[]
        if error<.105:self.done=True;state='completed'
        elif elapsed>self.timeout:self.done=True;state='aborted'
        elif front is None or near is None:state='INVALID_SCAN'
        elif near<.145 or front<.20:state='LASER_STOP'
        elif self.path is not None:
            self.progress=self.path.project(x,y,self.progress,.7)
            if self.target is not None and math.dist(pose[:2],self.target)<.19 and self.plan.kind!='GOAL':
                if not self.visits or math.dist(self.visits[-1],self.target)>.12:self.visits.append(self.target)
                self.target=None
            carrot=None
            lengths=[.34,.26,.18,.10] if abs(self.lookahead-.34)<1e-8 else [self.lookahead,self.lookahead*.76,self.lookahead*.53,.10]
            for length in lengths:
                q=self.path.point_at(self.progress+length)
                if self.plan.grid.line_free(pose[:2],q,self.plan.grid.track):carrot=q;break
            if carrot is None:state='MAP_STOP'
            else:
                angle=wrap(math.atan2(carrot[1]-y,carrot[0]-x)-h);length=max(.05,math.dist(pose[:2],carrot))
                if abs(angle)>self.align_threshold:nv=0.;nw=max(-1.4,min(1.4,2.2*angle));state='ALIGN'
                else:
                    curvature=2*math.sin(angle)/length
                    remaining=max(.0,self.path.total-self.progress)
                    nv=min(self.max_v,self.max_v/(1+self.curve_slowdown*abs(curvature)),math.sqrt(max(0.,front-.22)),max(.04,remaining/.9))
                    if abs(curvature)>1e-8:nv=min(nv,1.4/abs(curvature))
                    nw=nv*curvature;state='TRACK'
                v,w,prediction=self.local_command(pose,nv,nw,dt)
                if not prediction:state='LOCAL_STOP'
                elif v<.001 and nv>.01:state='LOCAL_BRAKE'
        if state in ['MAP_STOP','LOCAL_STOP','WAIT_PLAN']:
            w=max(self.w-self.angular_accel*dt,min(self.w+self.angular_accel*dt,0.))
        if math.dist(pose[:2],self.last_motion)>.10:self.last_motion=pose[:2];self.motion_at=now
        if now-self.motion_at>7. and not self.done:
            # Reobserve in place and reselect a target; never revert to wall following.
            if self.target is not None:self.visits.append(self.target);self.target=None
            if front is not None and near is not None and near>.15:
                v=0.;w=max(-1.0,self.w-self.angular_accel*dt);state='REOBSERVE'
            if now-self.motion_at>10.:self.motion_at=now;self.path=None
        self.v,self.w=float(v),float(w)
        cmd=Twist();cmd.linear.x=self.v;cmd.angular.z=self.w;self.pub.publish(cmd)
        if prediction:self.local_pub.publish(self.ros_path(prediction))
        data=dict(node=self.get_name(),algorithm='online_flood_fill',state=state,x=x,y=y,yaw=h,elapsed=elapsed,
                  v=self.v,w=self.w,front=front,scan_min=near,goal=list(self.goal),goal_error=error,replans=self.replans,
                  target=self.target,planning_mode=self.plan.kind if self.plan else None,scan_messages=self.scans,odom_messages=self.odoms,
                  known_cells=self.plan.known_cells if self.plan else 0,progress=self.progress,visited_targets=len(self.visits))
        self.status_pub.publish(String(data=json.dumps(data)))
        self.writer.writerow([elapsed,x,y,h,front,near,v,w,state,error,self.replans,*(self.target or ('','')),self.progress]);self.stream.flush()
        if state!=self.last_state:self.get_logger().info(json.dumps(data));self.last_state=state
        if self.done:(self.output/'flood_result.json').write_text(json.dumps(data,indent=2))

    def destroy_node(self):
        self.pub.publish(Twist());self.pool.shutdown(wait=True,cancel_futures=True)
        self.stream.close();self.events.close();return super().destroy_node()


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler);signal.signal(signal.SIGTERM,signal.default_int_handler)
    node=FloodNavigator()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if rclpy.ok():
            for _ in range(5):node.pub.publish(Twist());time.sleep(.03)
        node.destroy_node();rclpy.try_shutdown()
