"""Timed open-loop square (supplement to the Ch6.1 velocity commands)."""
import math
from .controller_base import ControllerBase, run_node


class OpenLoopController(ControllerBase):
    def __init__(self):
        super().__init__('tb3_open_loop')
        self.speed = self.number('speed', 0.2)
        self.turn_speed = self.number('turn_speed', 0.5)
        if self.speed > self.max_v or self.turn_speed > self.max_w:
            raise ValueError('speed/turn_speed exceeds max_v/max_w')
        self.straight_seconds = self.number('straight_seconds', 5.0)
        self.turn_seconds = self.number('turn_seconds', math.pi)
        self.pause_seconds = self.number('pause_seconds', 0.5)
        self.initial_pose = None
        self.timer = self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        if not self.ready():
            if self.started is not None and not self.done:
                self.done = True
                self.status('aborted', reason='odometry lost')
            return
        if self.done:
            self.command()
            return
        if self.initial_pose is None:
            self.initial_pose = (self.x, self.y, self.theta)
            x, y, yaw = self.initial_pose
            length = self.speed * self.straight_seconds
            points = [(0, 0), (length, 0), (length, length), (0, length), (0, 0)]
            self.set_reference([(x + a * math.cos(yaw) - b * math.sin(yaw),
                                 y + a * math.sin(yaw) + b * math.cos(yaw)) for a, b in points])
        elapsed = (self.get_clock().now().nanoseconds - self.started) / 1e9
        cycle = self.straight_seconds + self.turn_seconds + 2 * self.pause_seconds
        side = int(elapsed / cycle)
        phase = elapsed - side * cycle
        if side >= 4:
            self.finish(sides_completed=4, expected_side=self.speed * self.straight_seconds,
                        expected_turn=self.turn_speed * self.turn_seconds,
                        closure_error=math.dist((self.x, self.y), self.initial_pose[:2]))
            return
        if phase < self.straight_seconds:
            self.command(self.speed, 0.0)
            self.status(f'straight_{side + 1}')
        elif phase < self.straight_seconds + self.pause_seconds:
            self.command()
            self.status(f'vertex_{side + 1}')
        elif phase < self.straight_seconds + self.pause_seconds + self.turn_seconds:
            self.command(0.0, self.turn_speed)
            self.status(f'turn_{side + 1}')
        else:
            self.command()
            self.status(f'pause_{side + 1}')


def main(args=None):
    run_node(OpenLoopController, args)
