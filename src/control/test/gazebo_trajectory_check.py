#!/usr/bin/env python3
"""Actual Gazebo integration check. Use an isolated ROS domain/master and empty world.

source /opt/ros/humble/setup.bash && source install/setup.bash
ROS_DOMAIN_ID=174 GAZEBO_MASTER_URI=http://127.0.0.1:11375 python3 src/control/test/gazebo_trajectory_check.py
"""
import sys
import math
import time
from pathlib import Path
import rclpy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from interfaces.msg import LocalTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node('control_acceptance')
pose = None
stamp = 0.
joints = None
samples = []
joint_samples = []
blocks = []
def odom(msg):
    global pose, stamp
    pose = msg.pose.pose
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    samples.append((stamp, pose.position.x, pose.position.y, pose.position.z, 2*math.atan2(pose.orientation.z, pose.orientation.w)))
def state(msg):
    global joints
    joints = msg
    joint_samples.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, list(msg.position), list(msg.velocity)))
node.create_subscription(Odometry, '/robot/odom', odom, 100)
node.create_subscription(JointState, '/joint_states', state, 100)
pub = node.create_publisher(LocalTrajectory, '/local_trajectory', 10)
spawn = node.create_client(SpawnEntity, '/spawn_entity')
delete = node.create_client(DeleteEntity, '/delete_entity')
def spin_until(predicate, timeout=30):
    end = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > end:
            raise AssertionError('timeout waiting for Gazebo/ROS')
        rclpy.spin_once(node, timeout_sec=.02)
def call(client, request):
    assert client.wait_for_service(timeout_sec=20)
    future = client.call_async(request)
    spin_until(future.done)
    result = future.result()
    assert result.success, result.status_message

def robot(x=0., y=0.):
    global pose, joints
    req = DeleteEntity.Request(); req.name = 'xyz_test_robot'
    assert delete.wait_for_service(timeout_sec=20)
    future = delete.call_async(req); spin_until(future.done)
    for _ in range(10): rclpy.spin_once(node, timeout_sec=.01)
    pose = None; joints = None
    req = SpawnEntity.Request(); req.name = 'xyz_test_robot'
    req.xml = (Path(__file__).resolve().parents[1] / 'urdf/robot.urdf').read_text()
    req.initial_pose.position.x = x; req.initial_pose.position.y = y
    req.initial_pose.position.z = .32
    call(spawn, req)
    spin_until(lambda: pose is not None and joints is not None and pub.get_subscription_count() > 0)

def run(label, mode, points):
    msg = LocalTrajectory(); msg.header.frame_id = 'world'; msg.mode = mode
    for t, x, y, z, yaw in points:
        p = MultiDOFJointTrajectoryPoint(); tr = Transform()
        tr.translation.x = float(x); tr.translation.y = float(y); tr.translation.z = float(z)
        tr.rotation.z = math.sin(yaw / 2); tr.rotation.w = math.cos(yaw / 2)
        p.transforms = [tr]
        p.time_from_start.sec = int(t); p.time_from_start.nanosec = round((t - int(t)) * 1e9)
        msg.points.append(p)
    samples.clear()
    joint_samples.clear()
    start = stamp
    switching = abs(pose.position.z - points[0][3]) > .04
    pub.publish(msg)
    spin_until(lambda: stamp >= start + points[-1][0] + (1.5 if switching else 0) + 2, timeout=max(240, points[-1][0]*5))
    goal = points[-1]
    error = math.dist((pose.position.x, pose.position.y, pose.position.z), goal[1:4])
    yaw = 2 * math.atan2(pose.orientation.z, pose.orientation.w)
    assert error < .02 and abs(math.remainder(yaw - goal[4], 2*math.pi)) < .02, (label, error, pose)
    # Every reported base sample must lie close to the time-parameterized polyline.
    moving = [s for s in samples if abs(s[1]-points[0][1]) + abs(s[2]-points[0][2]) > .002]
    assert moving or points[-1][1:4] == points[0][1:4]
    assert all(math.isfinite(v) for v in joints.position + joints.velocity)
    if mode == 'foot':
        by_time = {round(row[0], 6): row for row in samples}
        penetration = 0.
        contacts = [set(), set()]
        for ts, q, velocity in joint_samples:
            row = by_time.get(round(ts, 6))
            if row is None: continue
            _, bx, by, bz, yaw = row
            for side in range(2):
                j = 3 + side * 4
                a = 1.15 + q[j+1]; b = a - 2.3 + q[j+2]
                x = -.28 * (math.sin(a) + math.sin(b))
                lx = .04 + x * math.cos(q[j])
                ly = (.34 if side == 0 else -.34) + x * math.sin(q[j])
                fx = bx + lx*math.cos(yaw) - ly*math.sin(yaw)
                fy = by + lx*math.sin(yaw) + ly*math.cos(yaw)
                fz = bz + .06 - .28*(math.cos(a)+math.cos(b)) - .0175
                for dx in [-.0175,.0725,.1625]:
                    for dy in [-.05,0,.05]:
                        angle = yaw + q[j]
                        wx = fx + dx*math.cos(angle)-dy*math.sin(angle)
                        wy = fy + dx*math.sin(angle)+dy*math.cos(angle)
                        height = max([0.] + [h for x,w,h in blocks if x < wx < x+w and abs(wy)<1])
                        penetration = max(penetration, height - fz)
                height = max([0.] + [h for x,w,h in blocks if x < fx < x+w and abs(fy)<1])
                if abs(fz-height)<.002: contacts[side].add(round(height,2))
            assert all(abs(v) <= (1.01 if i==0 else 12.01) for i,v in enumerate(velocity)), (label, velocity)
        assert penetration < .005, (label, 'foot penetration', penetration)
        if blocks:
            expected = {round(h,2) for _,_,h in blocks}
            assert all(expected <= c for c in contacts), (label,'missing foot contacts',contacts)
        print(f'  foot penetration <= {penetration:.6f} m; contacts={contacts}', flush=True)
    print(f'PASS {label}: endpoint error={error:.6f} m; samples={len(samples)}', flush=True)

def stairs():
    global blocks
    assert delete.wait_for_service(timeout_sec=20)
    # Each box is a real Gazebo collision surface; no terrain data is sent to Control.
    blocks = [(.5,.4,.08),(.9,.45,.20),(1.35,.5,.36),
              (1.85,.4,.26),(2.25,.45,.12),(2.7,.5,0.0)]
    for i,(x,width,height) in enumerate(blocks):
        if height == 0: continue
        req = DeleteEntity.Request(); req.name = f'xyz_step_{i}'
        future = delete.call_async(req); spin_until(future.done)
        req = SpawnEntity.Request(); req.name = f'xyz_step_{i}'
        req.xml = f'<sdf version="1.6"><model name="step"><static>true</static><link name="surface"><collision name="box"><geometry><box><size>{width} 2 {height}</size></box></geometry></collision><visual name="box"><geometry><box><size>{width} 2 {height}</size></box></geometry></visual></link></model></sdf>'
        req.initial_pose.position.x = x + width/2
        req.initial_pose.position.z = height/2
        call(spawn,req)
    robot()
    points = [(0,0,0,.44,0)]
    old = 0.
    for x,width,height in blocks:
        points.append(((x-.22)/.04,x-.22,0,.44+old,0))
        points.append(((x+.06)/.04,x+.06,0,.44+height,0))
        old = height
    points.append((80,3.2,0,.44,0))
    run('continuous stairs: +8/+12/+16/-10/-14/-12 cm', 'foot', points)

def scene():
    robot(-8.,3.9)
    run('scene wheel approach','wheel',[(0,-8.,3.9,.32,0),(4,-7.6,3.9,.32,0)])
    route = [(0.,-7.6,3.9,.44,0.)]
    def add(x,y,z,yaw,seconds=None):
        t,px,py,pz,pyaw=route[-1]
        duration = seconds if seconds is not None else max(math.dist((x,y,z),(px,py,pz))/.04,abs(math.remainder(yaw-pyaw,2*math.pi))/.08, .5)
        route.append((t+duration,x,y,z,yaw))
    # Existing south flight: ten 0.16 m risers, 0.30 m treads, then half landing.
    for i in range(10):
        edge = -7.1 + .3*i
        add(edge-.22,3.9,.44+.16*i,0)
        add(edge+.06,3.9,.44+.16*(i+1),0)
    add(-3.45,3.9,2.04,0)
    add(-3.45,3.9,2.04,math.pi/2)
    add(-3.45,5.65,2.04,math.pi/2)
    add(-3.45,5.65,2.04,math.pi)
    for i in range(10):
        edge=-4.1-.3*i
        add(edge+.22,5.65,2.04+.16*i,math.pi)
        add(edge-.06,5.65,2.04+.16*(i+1),math.pi)
    add(-7.75,5.65,3.64,math.pi)
    add(-7.75,5.65,3.64,0)
    for i in range(10):
        edge=-6.8+.3*i
        add(edge-.22,5.65,3.64-.16*i,0)
        add(edge+.06,5.65,3.64-.16*(i+1),0)
    add(-3.45,5.65,2.04,0)
    add(-3.45,5.65,2.04,-math.pi/2)
    add(-3.45,3.9,2.04,-math.pi/2)
    add(-3.45,3.9,2.04,math.pi)
    for i in range(10):
        edge=-4.4-.3*i
        add(edge+.22,3.9,2.04-.16*i,math.pi)
        add(edge-.06,3.9,2.04-.16*(i+1),math.pi)
    add(-7.6,3.9,.44,math.pi)
    print(f'Sending ONE foot LocalTrajectory: {len(route)} points, {route[-1][0]:.1f} simulation seconds',flush=True)
    run('scene floor 1 -> floor 2 -> floor 1, all foot', 'foot', route)
    run('scene foot to wheel / exit', 'wheel',[(0,-7.6,3.9,.32,math.pi),(4,-8.,3.9,.32,math.pi)])

try:
    if '--scene' in sys.argv:
        scene()
    elif '--stairs' in sys.argv:
        stairs()
    else:
        robot()
        run('wheel forward', 'wheel', [(0,0,0,.32,0),(3,.3,0,.32,0)])
        run('wheel to foot / flat forward', 'foot', [(0,.3,0,.44,0),(5,.6,0,.44,0)])
        run('foot backward', 'foot', [(0,.6,0,.44,0),(5,.3,0,.44,0)])
        run('foot yaw', 'foot', [(0,.3,0,.44,0),(5,.3,0,.44,.4)])
        run('foot to wheel', 'wheel', [(0,.3,0,.32,.4),(2,.3,0,.32,.4)])
        print('FLAT REGRESSION COMPLETE', flush=True)
finally:
    node.destroy_node(); rclpy.shutdown()
