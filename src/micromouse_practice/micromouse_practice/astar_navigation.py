"""Known-map A* navigation with odometry tracking and local laser protection."""
import csv
import json
import math
from pathlib import Path
import signal
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from tf2_ros import StaticTransformBroadcaster
from maze_control.controller_base import latched_qos
from maze_control.control_math import Polyline, wrap
from .grid_planner import GridMap
from .laser import sector
from .step_visualization import StepVisualization


class AStarNavigator(Node):
    def __init__(self):
        super().__init__('tb3_astar_navigation')
        name=self.declare_parameter('map','migong').value
        self.timeout=float(self.declare_parameter('timeout',600.).value)
        visualize=bool(self.declare_parameter('visualize',False).value)
        visual_port=int(self.declare_parameter('visual_port',8765).value)
        self.output=Path(self.declare_parameter('output_dir','/tmp/astar').value).expanduser()
        self.output.mkdir(parents=True,exist_ok=True)
        root=Path(get_package_share_directory('micromouse_practice'))
        self.cfg=json.loads((root/'config/maps.json').read_text())[name]
        self.grid=GridMap(root/'worlds'/self.cfg['world'],self.cfg)
        # The plan keeps a larger margin; bounded tracking error may use part of
        # that reserve while retaining a separate physical clearance envelope.
        tracking_margin=float(self.cfg['planning'].get('tracking_margin',.05))
        if not 0<tracking_margin<=self.grid.margin:
            raise ValueError('Tracking margin must be positive and no larger than planning margin')
        self.track_mask=(self.grid.distance<=self.grid.radius+tracking_margin+self.grid.res/math.sqrt(2))|~self.grid.domain
        self.plan=self.grid.plan(self.cfg['start'],self.cfg['goal'],self.output,record_trace=visualize)
        self.path=Polyline(self.plan['path']); self.progress=0.
        self.max_v=float(self.cfg['planning'].get('max_v',.5))
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.status_pub=self.create_publisher(String,'/control_status',latched_qos())
        self.reference_pub=self.create_publisher(RosPath,'/reference_path',latched_qos())
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',latched_qos())
        self.inflated_pub=self.create_publisher(OccupancyGrid,'/planning_grid',latched_qos())
        self.tf=StaticTransformBroadcaster(self)
        self.pose=self.anchor=self.scan=None
        self.scan_at=self.odom_at=-math.inf
        self.scans=self.odoms=0; self.started=None; self.done=False; self.last_state=None
        self.pause_total=0.; self.paused_since=None; self.visual=None
        self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.stream=(self.output/'navigation_telemetry.csv').open('w',newline='')
        self.writer=csv.writer(self.stream)
        self.writer.writerow(['t','x','y','yaw','front','left','right','scan_min','v','w','state','progress','goal_error'])
        if visualize:
            self.visual=StepVisualization(self.grid,self.plan,root/'assets/astar_steps.html',self.output,visual_port)
            self.get_logger().info(f'Step visualization: http://127.0.0.1:{self.visual.port} (or use the VM IP); waiting for Start driving')
        self.create_timer(.05,self.control)
        self.create_timer(.10,self.watchdog,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_grid(self.map_pub,self.grid.occupied)
        self.publish_grid(self.inflated_pub,self.grid.blocked)
        self.get_logger().info(f"A*: {self.plan['path_length_m']:.2f} m, {self.plan['expanded_nodes']} expanded cells, {self.plan['planning_seconds']:.3f} s")

    def publish_grid(self,pub,mask):
        msg=OccupancyGrid(); msg.header.frame_id='map'; msg.header.stamp=self.get_clock().now().to_msg()
        msg.info.resolution=self.grid.res; msg.info.width=self.grid.width; msg.info.height=self.grid.height
        msg.info.origin.position.x,msg.info.origin.position.y=self.grid.origin
        msg.info.origin.orientation.w=1.
        msg.data=(mask.astype('int8')*100).ravel().tolist(); pub.publish(msg)

    def publish_frames(self,frame):
        ax,ay,ah=self.anchor; sx,sy=self.cfg['start']; c,s=math.cos(ah),math.sin(ah)
        tf=TransformStamped(); tf.header.frame_id='map'; tf.child_frame_id=frame
        tf.header.stamp=self.get_clock().now().to_msg()
        tf.transform.translation.x=sx-c*ax-s*ay
        tf.transform.translation.y=sy+s*ax-c*ay
        tf.transform.rotation.z=math.sin(-ah/2); tf.transform.rotation.w=math.cos(-ah/2)
        self.tf.sendTransform(tf)
        # Publish the reference in odom so the existing odometry logger can also
        # overlay it correctly; RViz transforms both into the map frame.
        path=RosPath(); path.header.frame_id=frame; path.header.stamp=tf.header.stamp
        for x,y in self.plan['path']:
            p=PoseStamped(); p.header=path.header
            dx,dy=x-sx,y-sy
            p.pose.position.x=ax+c*dx-s*dy; p.pose.position.y=ay+s*dx+c*dy
            p.pose.orientation.w=1.; path.poses.append(p)
        self.reference_pub.publish(path)

    def odom_cb(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        h=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if not all(math.isfinite(v) for v in [p.x,p.y,h]): return
        if self.anchor is None:
            self.anchor=(p.x,p.y,h); self.publish_frames(msg.header.frame_id or 'odom')
        ax,ay,ah=self.anchor; dx,dy=p.x-ax,p.y-ay
        self.pose=(self.cfg['start'][0]+math.cos(ah)*dx+math.sin(ah)*dy,
                   self.cfg['start'][1]-math.sin(ah)*dx+math.cos(ah)*dy,wrap(h-ah))
        self.odom_at=time.monotonic(); self.odoms+=1

    def scan_cb(self,msg):
        self.scan=msg; self.scan_at=time.monotonic(); self.scans+=1

    def watchdog(self):
        if self.done or (self.visual is not None and not self.visual.is_running()) or time.monotonic()-min(self.scan_at,self.odom_at)>.7:
            self.pub.publish(Twist())

    def motion_free(self,x,y,h,v,w):
        previous=(x,y)
        for _ in range(12):
            dt=.05
            if abs(w)<1e-8: nx,ny=x+v*dt*math.cos(h),y+v*dt*math.sin(h)
            else:
                nx=x+v/w*(math.sin(h+w*dt)-math.sin(h))
                ny=y-v/w*(math.cos(h+w*dt)-math.cos(h))
            if not self.grid.line_free(previous,(nx,ny),self.track_mask): return False
            x,y,h=nx,ny,h+w*dt; previous=(x,y)
        return True

    def control(self):
        if self.done: return
        if self.scan is None or self.pose is None or time.monotonic()-min(self.scan_at,self.odom_at)>.7:
            self.pub.publish(Twist()); return
        now=self.get_clock().now().nanoseconds/1e9
        x,y,h=self.pose
        front,left,right=[sector(self.scan,*a) for a in [(-18,18),(60,100),(-100,-60)]]
        nearest=sector(self.scan,-180,180)
        if self.visual is not None and not self.visual.is_running():
            if self.started is not None and self.paused_since is None: self.paused_since=now
            elapsed=0. if self.started is None else self.paused_since-self.started-self.pause_total
            state='WAIT_VISUAL_START' if self.started is None else 'PAUSED_VISUAL'
            data=dict(node=self.get_name(),state=state,map=self.cfg['world'],algorithm='grid_astar',
                      x=x,y=y,yaw=h,elapsed=elapsed,front=front,left=left,right=right,scan_min=nearest,
                      v=0.,w=0.,scan_messages=self.scans,odom_messages=self.odoms,progress=self.progress,
                      planned_length=self.path.total,goal_error=math.dist((x,y),self.cfg['goal']),target=None)
            self.pub.publish(Twist()); self.visual.update(data)
            self.status_pub.publish(String(data=json.dumps(data)))
            if self.last_state!=state:
                self.get_logger().info(state); self.last_state=state
            return
        if self.paused_since is not None:
            self.pause_total+=now-self.paused_since; self.paused_since=None
        if self.started is None: self.started=now
        elapsed=now-self.started-self.pause_total
        self.progress=self.path.project(x,y,self.progress,.65)
        error=math.dist((x,y),self.cfg['goal'])
        v=w=0.; state='TRACK'; target=None; curvature=0.
        if error<.085 and self.progress>self.path.total-.20:
            self.done=True; state='completed'
        elif elapsed>self.timeout:
            self.done=True; state='aborted'
        elif front is None or nearest is None:
            state='INVALID_SCAN'
        elif front<.24 or nearest<.155:
            state='OBSTACLE_STOP'
        else:
            target=None
            for lookahead in [.30,.22,.15,.08]:
                p=self.path.point_at(self.progress+lookahead)
                if self.grid.line_free((x,y),p,self.track_mask): target=p; break
            if target is None:
                state='MAP_STOP'
            else:
                dx,dy=target[0]-x,target[1]-y; distance=math.hypot(dx,dy)
                alpha=wrap(math.atan2(dy,dx)-h)
                if abs(alpha)>.55:
                    w=max(-1.4,min(1.4,2.5*alpha)); state='ALIGN'
                else:
                    curvature=2*math.sin(alpha)/max(distance,.05)
                    v=min(self.max_v,self.max_v/(1+1.1*abs(curvature)),
                          math.sqrt(max(0.,front-.26)),math.sqrt(max(0.,error)))
                    if abs(curvature)>1e-8: v=min(v,1.4/abs(curvature))
                    w=v*curvature
                    if not self.motion_free(x,y,h,v,w):
                        v=0.; w=max(-1.2,min(1.2,2.5*alpha)); state='MAP_ALIGN'
        msg=Twist(); msg.linear.x=float(v); msg.angular.z=float(w); self.pub.publish(msg)
        data=dict(node=self.get_name(),state=state,map=self.cfg['world'],algorithm='grid_astar',
                  x=x,y=y,yaw=h,elapsed=elapsed,front=front,left=left,right=right,scan_min=nearest,
                  v=v,w=w,scan_messages=self.scans,odom_messages=self.odoms,progress=self.progress,
                  planned_length=self.path.total,goal_error=error,target=target,curvature=curvature)
        self.status_pub.publish(String(data=json.dumps(data)))
        if self.visual is not None: self.visual.update(data)
        self.writer.writerow([elapsed,x,y,h,front,left,right,nearest,v,w,state,self.progress,error]); self.stream.flush()
        if state!=self.last_state:
            self.get_logger().info(json.dumps(data)); self.last_state=state
        if self.done:
            data['reason']='exit_reached' if state=='completed' else 'timeout'
            (self.output/'astar_result.json').write_text(json.dumps(data,indent=2))

    def destroy_node(self):
        self.pub.publish(Twist())
        if self.visual is not None: self.visual.close()
        self.stream.close(); return super().destroy_node()


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler)
    signal.signal(signal.SIGTERM,signal.default_int_handler)
    node=AStarNavigator()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        if rclpy.ok():
            for _ in range(5): node.pub.publish(Twist()); time.sleep(.03)
        node.destroy_node(); rclpy.try_shutdown()
