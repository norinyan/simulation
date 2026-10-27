#!/usr/bin/env python3
"""Position-triggered one-way pedestrians; publish only actual Gazebo states."""
import math
import json
from pathlib import Path
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, GetEntityState, SetEntityState, SpawnEntity
from interfaces.msg import DynamicObstacle, GlobalPath
from std_msgs.msg import String
import yaml


class DynamicObstacles(Node):
    def __init__(self):
        super().__init__('dynamic_obstacles')
        filename = self.declare_parameter('config_file', '').value
        config = yaml.safe_load(Path(filename).read_text())
        self.sources = []
        ids = set()
        for item in config['obstacles']:
            if not item.get('enabled', True):
                continue
            name = item['id']
            size = [float(v) for v in item['size']]
            route = [[float(v) for v in p] for p in item['route']]
            speed = float(item['speed'])
            hold = float(item.get('hold_time', 0.0))
            triggers = item['triggers']
            for trigger in triggers:
                lower, upper, direction = trigger['min'], trigger['max'], trigger['direction']
                if (len(lower) != 3 or len(upper) != 3 or len(direction) != 2 or
                        not all(math.isfinite(v) for v in lower+upper+direction) or
                        any(a >= b for a, b in zip(lower, upper)) or math.hypot(*direction) < 1e-9):
                    raise ValueError('Invalid trigger: ' + name)
            lane = item.get('adapt_lane')
            if lane and (not all(math.isfinite(lane[k]) for k in ('ground_z', 'at_x', 'min', 'max')) or
                         lane['min'] > lane['max'] or len(route) != 4):
                raise ValueError('Invalid corridor lane: ' + name)
            values = size + [v for point in route for v in point] + [speed, hold]
            if (not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name) or name in ids or
                    len(size) != 3 or len(route) < 2 or any(len(p) != 3 for p in route) or
                    not all(math.isfinite(v) for v in values) or min(size) <= 0 or
                    speed <= 0 or hold < 0 or not triggers or
                    any(abs(p[2]-route[0][2]) > 1e-9 for p in route)):
                raise ValueError('Invalid pedestrian source: ' + name)
            ids.add(name)
            self.sources.append(dict(id=name, size=size, route=route, base_route=route,
                                     speed=speed, hold=hold, triggers=triggers, lane=lane,
                                     fired_task=-1, mode='absent', instance=None, sequence=0,
                                     started=None, control_pending=None, observation_pending=None,
                                     actual=None, actual_time=None, message=None, at_end=False,
                                     next_control=0.0, next_observe=0.0,
                                     next_delete=0.0, last_error=-math.inf))
        if not self.sources:
            raise ValueError('No enabled pedestrian sources')
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete = self.create_client(DeleteEntity, '/delete_entity')
        self.set_state = self.create_client(SetEntityState, '/set_entity_state')
        self.get_state = self.create_client(GetEntityState, '/get_entity_state')
        self.publisher = self.create_publisher(
            DynamicObstacle, '/dynamic_obstacles',
            QoSProfile(depth=128, reliability=ReliabilityPolicy.RELIABLE))
        self.task_active = False
        self.task_id = 0
        self.task_armed = False
        self.last_status = None
        self.path = None
        self.path_sub = self.create_subscription(GlobalPath, '/global_path', self.on_path,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_sub = self.create_subscription(String, '/local_planning/status', self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.robot = None
        self.robot_time = None
        self.robot_query = None
        self.next_snapshot = 0.0
        self.next_robot_query = 0.0
        # Schedule 20 Hz operations independently of service callback latency.
        self.timer = self.create_timer(0.01, self.update)

    def on_path(self, message):
        if self.task_active:
            return  # Match the local node: it refuses a new path during execution.
        self.path = message if message.header.frame_id == 'world' and message.points else None
        self.task_armed = self.path is not None
        # Path and status have different publishers; accept either arrival order.
        if self.last_status in ('planning', 'executing', 'avoiding', 'waiting', 'stale'):
            self.on_status(String(data=json.dumps({'state': self.last_status})))

    def on_status(self, message):
        try:
            state = json.loads(message.data)['state']
        except (ValueError, KeyError, TypeError):
            return
        if (state in ('planning', 'executing', 'avoiding', 'waiting', 'stale') and
                not self.task_active and self.task_armed):
            self.task_id += 1
            self.task_active = True
            self.task_armed = False
        elif state in ('idle', 'ready', 'failed', 'completed'):
            self.task_active = False
        self.last_status = state

    def match_trigger(self, source, now):
        if (not self.task_active or source['fired_task'] == self.task_id or
                self.robot is None or self.robot_time is None or
                not 0 <= now-self.robot_time <= .15):
            return None
        p, v = self.robot.pose.position, self.robot.twist.linear
        for trigger in source['triggers']:
            if (all(a <= x <= b for x, a, b in zip((p.x, p.y, p.z), trigger['min'], trigger['max'])) and
                    (v.x*trigger['direction'][0]+v.y*trigger['direction'][1]) /
                    math.hypot(*trigger['direction']) > .05):
                return trigger
        return None

    def prepare_route(self, source, trigger):
        # Select once before birth, never alter an in-flight pedestrian route.
        route = [list(point) for point in source['base_route']]
        lane = source['lane']
        if lane:
            candidates = [] if self.path is None else [p.position for p in self.path.points
                if abs(p.position.z-lane['ground_z']) < .05 and
                all(math.isfinite(v) for v in (p.position.x, p.position.y, p.position.z))]
            if not candidates:
                return False
            point = min(candidates, key=lambda p: abs(p.x-lane['at_x']))
            if abs(point.x-lane['at_x']) > .25:
                return False
            route[1][1] = route[2][1] = min(lane['max'], max(lane['min'], point.y))
        if trigger.get('reverse_route', False):
            route.reverse()
        legs, elapsed = [], 0.0
        for i, (a, b) in enumerate(zip(route, route[1:])):
            duration = math.dist(a, b)/source['speed']
            if duration <= 1e-6:
                raise ValueError('Repeated route point: ' + source['id'])
            legs.append((elapsed, elapsed+duration, a, b))
            elapsed += duration
            if i+1 < len(route)-1:
                elapsed += source['hold']
        source['route'], source['legs'], source['duration'] = route, legs, elapsed
        return True

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def set_vector(target, values):
        target.x, target.y, target.z = values

    @staticmethod
    def sdf(source):
        x, y, z = source['size']
        return f'''<sdf version="1.6"><model name="{source['instance']}">
          <static>false</static><link name="body"><gravity>false</gravity>
          <kinematic>true</kinematic><inertial><mass>1</mass><inertia>
          <ixx>{(y*y+z*z)/12}</ixx><iyy>{(x*x+z*z)/12}</iyy>
          <izz>{(x*x+y*y)/12}</izz></inertia></inertial>
          <collision name="box"><geometry><box><size>{x} {y} {z}</size></box></geometry></collision>
          <visual name="box"><geometry><box><size>{x} {y} {z}</size></box></geometry>
          <material><ambient>0.9 0.2 0.15 1</ambient><diffuse>0.9 0.2 0.15 1</diffuse></material>
          </visual></link></model></sdf>'''

    def empty_message(self, name=''):
        message = DynamicObstacle()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'world'
        message.id, message.active = name, False
        return message

    def refresh_robot(self):
        if self.robot_query is None and self.now() >= self.next_robot_query:
            self.next_robot_query = self.now()+.05
            request = GetEntityState.Request()
            request.name, request.reference_frame = 'ideal_robot', 'world'
            self.robot_query = self.get_state.call_async(request)
        elif self.robot_query is not None and self.robot_query.done():
            result = self.robot_query.result()
            self.robot_query = None
            previous, previous_time = self.robot, self.robot_time
            self.robot = result.state if result.success else None
            self.robot_time = (result.header.stamp.sec+result.header.stamp.nanosec*1e-9
                               if result.success else None)
            # The ideal executor sets poses and zeros Gazebo physics velocity.
            # Infer world motion from actual samples, not that zero twist or yaw.
            if self.robot is not None:
                velocity = self.robot.twist.linear
                velocity.x = velocity.y = velocity.z = 0.0
                if (previous is not None and previous_time is not None and
                        0 < self.robot_time-previous_time <= .15):
                    dt = self.robot_time-previous_time
                    for axis in ('x', 'y', 'z'):
                        setattr(velocity, axis, (getattr(self.robot.pose.position, axis)-
                                                getattr(previous.pose.position, axis))/dt)

    def birth_clear(self, source, now):
        # Never spawn from a stale robot sample or unknown neighbouring entity.
        if self.robot is None or self.robot_time is None or not 0 <= now-self.robot_time <= .15:
            return False
        a, size = source['route'][0], source['size']
        p = self.robot.pose.position
        gap = math.sqrt(sum(max(0.0, abs(v-q)-h/2)**2
                            for v, q, h in zip((p.x, p.y, p.z), a, size)))
        if gap < 1.2:
            return False
        for other in self.sources:
            if other is source or other['mode'] == 'absent':
                continue
            if other['actual'] is None or not 0 <= now-other['actual_time'] <= .15:
                return False
            p = other['actual'].pose.position
            if all(abs(v-q) <= (h+k)/2+.05 for v, q, h, k in
                   zip((p.x, p.y, p.z), a, size, other['size'])):
                return False
        return True

    @staticmethod
    def motion(source, elapsed):
        previous = source['route'][0]
        for begin, end, a, b in source['legs']:
            if elapsed < begin:
                return previous, (0.0, 0.0, 0.0), False
            if elapsed < end:
                ratio = max(0.0, (elapsed-begin)/(end-begin))
                return ([a[i]+ratio*(b[i]-a[i]) for i in range(3)],
                        [(b[i]-a[i])/(end-begin) for i in range(3)], False)
            previous = b
        return source['route'][-1], (0.0, 0.0, 0.0), True

    def update(self):
        if not all(c.service_is_ready() for c in (self.spawn, self.delete, self.set_state, self.get_state)):
            return
        self.refresh_robot()
        now = self.now()
        for source in self.sources:
            # Each source's control and actual observation requests are independent.
            # In particular a slow delete cannot prevent observation of that same
            # live entity, or movement/observation of entities on other floors.
            if source['control_pending'] is None:
                if source['mode'] == 'absent':
                    trigger = self.match_trigger(source, now)
                    if trigger is not None and self.prepare_route(source, trigger) and self.birth_clear(source, now):
                        source['fired_task'] = self.task_id
                        self.get_logger().info(f"Pedestrian triggered: {source['id']} task={self.task_id} sim={now:.3f}")
                        source['sequence'] += 1
                        source['instance'] = f"{source['id']}_{source['sequence']:03d}"
                        source['mode'] = 'spawning'
                        request = SpawnEntity.Request()
                        request.name, request.reference_frame = source['instance'], 'world'
                        request.xml = self.sdf(source)
                        self.set_vector(request.initial_pose.position, source['route'][0])
                        request.initial_pose.orientation.w = 1.0
                        source['control_pending'] = self.spawn.call_async(request)
                        source['control_pending'].add_done_callback(lambda f, s=source: self.spawned(s, f))
                elif source['mode'] == 'moving' and now >= source['next_control']:
                    position, velocity, arrived = self.motion(source, now-source['started'])
                    state = EntityState()
                    state.name, state.reference_frame = source['instance'], 'world'
                    self.set_vector(state.pose.position, position)
                    state.pose.orientation.w = 1.0
                    self.set_vector(state.twist.linear, velocity)
                    request = SetEntityState.Request()
                    request.state = state
                    source['next_control'] = now+.05
                    source['control_pending'] = self.set_state.call_async(request)
                    source['control_pending'].add_done_callback(lambda f, s=source, a=arrived: self.moved(s, f, a))
                elif source['mode'] == 'deleting' and now >= source['next_delete']:
                    request = DeleteEntity.Request()
                    request.name = source['instance']
                    source['control_pending'] = self.delete.call_async(request)
                    source['control_pending'].add_done_callback(lambda f, s=source: self.deleted(s, f))
            if (source['mode'] in ('moving', 'deleting') and
                    source['observation_pending'] is None and now >= source['next_observe']):
                self.observe(source)
        if now >= self.next_snapshot:
            self.next_snapshot = now+.05
            self.publish_snapshot()

    def publish_snapshot(self):
        # Replaying preserves the actual GetEntityState sampling timestamp.
        # Consumers separately check each live observation's freshness; a live
        # publisher heartbeat cannot make failed/stale entity observations safe.
        for source in self.sources:
            if source['message'] is not None:
                self.publisher.publish(source['message'])
        # A confirmed birth must be observed before declaring a complete live set.
        if not any(s['mode'] in ('moving', 'deleting') and s['message'] is None
                   for s in self.sources):
            self.publisher.publish(self.empty_message())

    def spawned(self, source, future):
        source['control_pending'] = None
        result = future.result()
        if not result.success:
            # Do not invent an observation or reuse a possibly existing instance ID.
            raise RuntimeError('Failed to spawn ' + source['instance'] + ': ' + result.status_message)
        source['mode'], source['started'] = 'moving', self.now()
        self.get_logger().info('Pedestrian entered: ' + source['instance'])
        self.observe(source)

    def moved(self, source, future, arrived):
        source['control_pending'] = None
        if not future.result().success:
            raise RuntimeError('Failed to move pedestrian: ' + source['instance'])
        source['at_end'] = arrived

    def observe(self, source):
        request = GetEntityState.Request()
        request.name, request.reference_frame = source['instance'], 'world'
        source['next_observe'] = self.now()+.05
        source['observation_pending'] = self.get_state.call_async(request)
        source['observation_pending'].add_done_callback(
            lambda f, s=source, name=source['instance']: self.observed(s, f, name))

    def observed(self, source, future, name):
        if source['observation_pending'] is future:
            source['observation_pending'] = None
        # A Get can finish after successful deletion. Never revive the departed
        # ID, and never let that old response overwrite a subsequent instance.
        if source['instance'] != name or source['mode'] not in ('moving', 'deleting'):
            return
        result = future.result()
        if not result.success:
            if self.now()-source['last_error'] >= 1.0:
                self.get_logger().error('Failed to observe pedestrian: ' + source['instance'])
                source['last_error'] = self.now()
            return
        source['actual'] = result.state
        source['actual_time'] = result.header.stamp.sec+result.header.stamp.nanosec*1e-9
        message = DynamicObstacle()
        message.header = result.header
        message.header.frame_id = 'world'
        message.id, message.active = source['instance'], True
        message.pose, message.twist = result.state.pose, result.state.twist
        self.set_vector(message.size, source['size'])
        source['message'] = message
        if source['at_end'] and source['mode'] == 'moving':
            p, v = result.state.pose.position, result.state.twist.linear
            if (math.dist((p.x, p.y, p.z), source['route'][-1]) <= 1e-6 and
                    math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z) <= 1e-6):
                source['mode'], source['next_delete'] = 'deleting', self.now()

    def deleted(self, source, future):
        source['control_pending'] = None
        result = future.result()
        if not result.success:
            self.get_logger().error('Failed to delete ' + source['instance'] + ': ' + result.status_message)
            source['next_delete'] = self.now()+.5
            return
        self.publisher.publish(self.empty_message(source['instance']))
        self.get_logger().info('Pedestrian exited: ' + source['instance'])
        source['mode'], source['instance'], source['actual'] = 'absent', None, None
        source['started'], source['actual_time'] = None, None
        source['message'], source['observation_pending'], source['at_end'] = None, None, False


def main():
    rclpy.init()
    node = DynamicObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
