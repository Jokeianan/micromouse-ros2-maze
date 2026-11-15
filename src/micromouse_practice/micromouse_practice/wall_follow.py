"""Laser-driven left-wall following; odometry handles entry and exit stages."""
import csv
import json
import math
import signal
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from maze_control.controller_base import latched_qos
from .laser import sector, command,predictive_velocity


class WallFollower(Node):
    def __init__(self):
        super().__init__('tb3_wall_follow')
        self.map = self.declare_parameter('map','migong').value
        self.algorithm = self.declare_parameter('algorithm','sprint').value
        self.mode = self.declare_parameter('mode','wall_follow').value
        self.require_map = self.declare_parameter('require_map',False).value
        self.map_at = -math.inf
        self.timeout = float(self.declare_parameter('timeout',240.).value)
        self.output = Path(self.declare_parameter('output_dir','/tmp/practice').value).expanduser()
        self.output.mkdir(parents=True,exist_ok=True)
        config = Path(get_package_share_directory('micromouse_practice'))/'config/maps.json'
        # This metadata is used ONLY for spawn/exit evaluation, never for steering.
        self.cfg = json.loads(config.read_text())[self.map]
        tuning=dict(flow_dist_gain=1.8,flow_angle_gain=2.0,flow_preview=.25,flow_curve_slowdown=2.3,
                    flow_speed_gain_reference=.22,flow_max_w=1.4,flow_slow_distance=1.,flow_front_stop=.32,
                    flow_front_release=.38,flow_side_release=.24,flow_clear_v=.07,flow_clear_w=-.85,
                    flow_acquire_v=.12,flow_acquire_w=.65,flow_linear_accel=.7,flow_linear_decel=1.,
                    flow_angular_accel=6.,flow_omega_filter=.70,flow_near_left=.24,
                    flow_prediction_time=.45,flow_min_center_clearance=.20)
        for key,value in tuning.items():self.cfg[key]=float(self.declare_parameter(key,value).value)
        self.velocity=self.angular=self.filtered_w=0.;self.previous_time=None
        self.entry_complete = 'entry_x' not in self.cfg
        self.exit_clearing = False
        self.pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.status_pub = self.create_publisher(String,'/control_status',latched_qos())
        self.scan_pub = self.create_publisher(String,'/scan_sectors',10)
        self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data)
        if self.require_map:
            self.create_subscription(OccupancyGrid,'/map',self.map_cb,latched_qos())
        self.scan = self.pose = self.anchor = None
        self.scan_pose=None
        self.scan_at = self.odom_at = -math.inf
        self.scans = self.odoms = 0
        self.done = False
        self.started = None
        self.last_state = None
        self.stream = (self.output/'wall_telemetry.csv').open('w',newline='')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(['t','x','y','yaw','front','left','right','scan_min','v','w','state'])
        self.create_timer(.05,self.control)
        self.create_timer(.10,self.watchdog,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def scan_cb(self,msg):
        self.scan,self.scan_at = msg,time.monotonic()
        self.scan_pose=self.pose
        self.scans += 1

    def map_cb(self,msg):
        # Only SLAM readiness is used; steering never reads map cells.
        if msg.info.width*msg.info.height==len(msg.data) and len(msg.data)>0:
            self.map_at=time.monotonic()

    def odom_cb(self,msg):
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        h = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if not all(math.isfinite(v) for v in [p.x,p.y,h]): return
        if self.anchor is None: self.anchor = (p.x,p.y,h)
        ax,ay,ah = self.anchor
        dx,dy = p.x-ax,p.y-ay
        self.pose = (self.cfg['start'][0]+math.cos(ah)*dx+math.sin(ah)*dy,
                     self.cfg['start'][1]-math.sin(ah)*dx+math.cos(ah)*dy,h-ah)
        self.odom_at = time.monotonic()
        self.odoms += 1

    def watchdog(self):
        if self.done or time.monotonic()-min(self.scan_at,self.odom_at)>.7 or (self.require_map and time.monotonic()-self.map_at>5.):
            self.pub.publish(Twist())
            if self.algorithm=='flow':self.velocity=self.angular=self.filtered_w=0.

    def control(self):
        if self.done: return
        if self.scan is None or self.pose is None or time.monotonic()-min(self.scan_at,self.odom_at)>.7 or (self.require_map and time.monotonic()-self.map_at>5.):
            self.pub.publish(Twist())
            return
        now = self.get_clock().now().nanoseconds/1e9
        dt=.05 if self.previous_time is None else max(.02,min(.1,now-self.previous_time));self.previous_time=now
        if self.started is None: self.started = now
        elapsed = now-self.started
        x,y,h = self.pose
        front,left,right = [sector(self.scan,*a) for a in [(-20,20),(60,100),(-100,-60)]]
        near = sector(self.scan,-180,180)
        ray45=sector(self.scan,43,47)
        ray90=sector(self.scan,88,92)
        near_left=sector(self.scan,25,75)
        v,w,state = command(front,left,ray45,ray90,near_left,self.algorithm,self.cfg)
        if self.algorithm=='flow' and state!='INVALID_SCAN':
            if self.last_state=='AVOID_RIGHT' and front<self.cfg['flow_front_release']:
                v,w,state=0.,-self.cfg['flow_max_w'],'AVOID_RIGHT'
            elif self.last_state=='LEFT_CLEARANCE' and near_left<self.cfg['flow_side_release'] and front>=self.cfg['flow_front_stop']:
                v,w,state=self.cfg['flow_clear_v'],self.cfg['flow_clear_w'],'LEFT_CLEARANCE'
        if not self.entry_complete:
            if x >= self.cfg['entry_x']:
                self.entry_complete=True
            else:
                v,w,state=.14,max(-.5,min(.5,-1.8*math.atan2(math.sin(h),math.cos(h)))),'ENTER_MAZE'
                if front is None or front<.25: v=0.
        gx,gy = self.cfg['goal']
        # Stop seeking the left wall once the known exit aperture is crossed.
        # This is a short exit-clearing motion, not a planned path through maze.
        if self.cfg.get('exit_entry_x') is not None:
            if x>=self.cfg['exit_entry_x'] and abs(y-gy)<.30 and front is not None and front>1.:
                self.exit_clearing=True
            if self.exit_clearing:
                v,w,state=.12,max(-.7,min(.7,-2.*math.atan2(math.sin(h),math.cos(h)))),'CLEAR_EXIT'
                if front is None or front<.25: v=0.
        tolerance=self.cfg.get('exit_tolerance',.85)
        reached = (x>=gx-.08 and abs(y-gy)<tolerance) if self.cfg['exit_axis']=='x' else (y>=gy-.08 and abs(x-gx)<tolerance)
        if self.mode == 'scan': v,w,state = 0.,0.,'OBSERVE_SCAN'
        elif self.mode == 'reactive':
            v,w,state = (0.,-.65,'AVOID_RIGHT') if front is not None and front<.4 else (.15,0.,'FORWARD')
            if front is None: v,w,state=0.,0.,'INVALID_SCAN'
        if reached and self.mode=='wall_follow': self.done=True; v,w,state=0.,0.,'completed'
        elif elapsed>self.timeout: self.done=True; v,w,state=0.,0.,'aborted'
        if self.algorithm=='flow':
            emergency=self.done or state=='INVALID_SCAN' or (near is not None and near<.155)
            if emergency:v=0.
            else:v=max(self.velocity-self.cfg['flow_linear_decel']*dt,min(self.velocity+self.cfg['flow_linear_accel']*dt,v))
            if state=='AVOID_RIGHT':v=0.
            mix=self.cfg['flow_omega_filter']
            self.filtered_w+=mix*(w-self.filtered_w)
            w=max(self.angular-self.cfg['flow_angular_accel']*dt,min(self.angular+self.cfg['flow_angular_accel']*dt,self.filtered_w))
            if state=='LEFT_CLEARANCE' and w>-.15:v=0.
            v=predictive_velocity(self.scan,self.scan_pose,self.pose,v,w,self.cfg['flow_prediction_time'],self.cfg['flow_min_center_clearance'])
            if self.done or state=='INVALID_SCAN':w=0.;self.filtered_w=0.
            self.velocity,self.angular=float(v),float(w)
        msg=Twist(); msg.linear.x=float(v); msg.angular.z=float(w); self.pub.publish(msg)
        data=dict(node=self.get_name(),state=state,map=self.map,algorithm=self.algorithm,x=x,y=y,yaw=h,
                  elapsed=elapsed,front=front,left=left,right=right,scan_min=near,
                  v=v,w=w,scan_messages=self.scans,odom_messages=self.odoms)
        self.status_pub.publish(String(data=json.dumps(data)))
        self.scan_pub.publish(String(data=json.dumps({k:data[k] for k in ['front','left','right','scan_min']})))
        self.writer.writerow([elapsed,x,y,h,front,left,right,near,v,w,state]); self.stream.flush()
        if state!=self.last_state:
            self.get_logger().info(json.dumps(data)); self.last_state=state
        if self.done:
            data['reason']='exit_reached' if reached else 'timeout'
            (self.output/'wall_result.json').write_text(json.dumps(data,indent=2))

    def destroy_node(self):
        self.pub.publish(Twist())
        self.stream.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler)
    signal.signal(signal.SIGTERM,signal.default_int_handler)
    node=WallFollower()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        if rclpy.ok():
            for _ in range(5): node.pub.publish(Twist()); time.sleep(.03)
        node.destroy_node(); rclpy.try_shutdown()
