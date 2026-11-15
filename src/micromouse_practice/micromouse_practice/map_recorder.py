"""Save only sensor-built /map data; never open a world, prior grid, or truth topic."""
import csv
import json
import math
from pathlib import Path
import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
from maze_control.controller_base import latched_qos


def atomic(path, data):
    tmp = path.with_name(path.name+'.tmp')
    tmp.write_bytes(data)
    tmp.replace(path)


def save_grid(msg, folder, summary):
    """PGM rows run top-to-bottom, OccupancyGrid rows bottom-to-top."""
    folder.mkdir(parents=True, exist_ok=True)
    grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    pixels = np.full(grid.shape, 205, dtype=np.uint8)
    pixels[grid == 0] = 254
    pixels[grid >= 65] = 0
    pixels[(grid > 0) & (grid < 65)] = 205
    atomic(folder/'map.pgm', f'P5\n{msg.info.width} {msg.info.height}\n255\n'.encode()+np.flipud(pixels).tobytes())
    q = msg.info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    origin = [msg.info.origin.position.x, msg.info.origin.position.y, yaw]
    atomic(folder/'map.yaml', (
        f'image: map.pgm\nmode: trinary\nresolution: {msg.info.resolution}\n'
        f'origin: {origin}\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n').encode())
    atomic(folder/'map.json', json.dumps(dict(
        frame_id=msg.header.frame_id, width=msg.info.width, height=msg.info.height,
        resolution=msg.info.resolution, origin=origin, data=list(msg.data), **summary)).encode())


class MapRecorder(Node):
    def __init__(self):
        super().__init__('online_map_recorder')
        self.output = Path(self.declare_parameter('output_dir', '/tmp/practice').value).expanduser()/'mapping'
        self.output.mkdir(parents=True, exist_ok=True)
        self.latest = None
        self.count = 0
        self.first_stamp = None
        self.elapsed = 0.
        self.completed = False
        self.last_saved = -math.inf
        self.checkpoints = [60., 180., 300., 420.]
        self.snapshots = []
        self.csv = (self.output/'map_growth.csv').open('w', newline='')
        self.writer = csv.writer(self.csv)
        self.writer.writerow(['map_stamp', 'control_elapsed', 'width', 'height', 'known_cells', 'free_cells', 'occupied_cells', 'known_area_m2'])
        self.status_pub = self.create_publisher(String, '/mapping_status', latched_qos())
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, latched_qos())
        self.create_subscription(String, '/control_status', self.control_cb, latched_qos())
        self.create_service(Trigger, '/save_online_map', self.save_cb)

    def control_cb(self, msg):
        status = json.loads(msg.data)
        self.elapsed = status.get('elapsed', 0.)
        if status.get('state') == 'completed': self.completed = True

    def summary(self):
        m = self.latest
        a = np.asarray(m.data)
        known = int(np.count_nonzero(a >= 0))
        return dict(source='SLAM Toolbox /scan + odom TF; no world geometry or prior map',
                    map_messages=self.count, control_elapsed=self.elapsed, completed=self.completed,
                    map_stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                    known_cells=known, free_cells=int(np.count_nonzero(a == 0)),
                    occupied_cells=int(np.count_nonzero(a >= 65)), unknown_cells=int(np.count_nonzero(a < 0)),
                    known_area_m2=known*m.info.resolution**2)

    def map_cb(self, msg):
        if msg.info.width*msg.info.height != len(msg.data) or not msg.data: return
        self.latest = msg
        self.count += 1
        s = self.summary()
        if self.first_stamp is None: self.first_stamp = s['map_stamp']
        self.writer.writerow([s['map_stamp'], self.elapsed, msg.info.width, msg.info.height,
                              s['known_cells'], s['free_cells'], s['occupied_cells'], s['known_area_m2']])
        self.csv.flush()
        if self.count == 1: self.save('initial')
        for checkpoint in self.checkpoints[:]:
            if self.elapsed >= checkpoint:
                self.save(f'{int(checkpoint):04d}s')
                self.checkpoints.remove(checkpoint)
        if self.completed:
            self.save('final')
        elif s['map_stamp']-self.last_saved >= 5.:
            self.save()
        self.status_pub.publish(String(data=json.dumps(s)))

    def save(self, stage=None):
        if self.latest is None: return False
        s = self.summary()
        save_grid(self.latest, self.output, s)
        self.last_saved = s['map_stamp']
        if stage:
            save_grid(self.latest, self.output/'stages'/stage, s)
            if stage not in self.snapshots: self.snapshots.append(stage)
            self.get_logger().info(f'Saved {stage}: {s["known_area_m2"]:.2f} m2 observed')
        s['stages'] = self.snapshots
        atomic(self.output/'mapping_result.json', json.dumps(s, indent=2).encode())
        return True

    def save_cb(self, request, response):
        response.success = self.save()
        response.message = str(self.output/'map.yaml') if response.success else 'No map received yet'
        return response

    def destroy_node(self):
        self.save('final' if self.completed else 'stopped')
        self.csv.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = MapRecorder()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
