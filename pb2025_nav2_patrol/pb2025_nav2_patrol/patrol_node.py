import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
import math
import threading
import os
import yaml
import subprocess
import time
from std_msgs.msg import Bool
from std_srvs.srv import Empty


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)


class PatrolNode(Node):
    def __init__(self):
        super().__init__('pb2025_nav2_patrol')

        # parameters
        self.declare_parameter('waypoints', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('loop', True)
        self.declare_parameter('wait_at_point', 0.0)
        self.declare_parameter('retry_on_fail', False)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('action_name', 'navigate_to_pose')

        self.waypoints = self.get_parameter('waypoints').get_parameter_value().string_array_value if \
            self.get_parameter('waypoints').type_ == rclpy.parameter.Parameter.Type.STRING_ARRAY else self.get_parameter('waypoints').value
        self.loop = self.get_parameter('loop').value
        self.wait_at_point = float(self.get_parameter('wait_at_point').value)
        self.retry_on_fail = self.get_parameter('retry_on_fail').value
        self.frame_id = self.get_parameter('frame_id').value
        self.auto_start = self.get_parameter('auto_start').value
        self.action_name = self.get_parameter('action_name').value

        # normalize waypoints if provided via params
        self.waypoints = self._normalize_waypoints(self.waypoints)

        # action client
        self._cb_group = ReentrantCallbackGroup()
        self._client = ActionClient(self, NavigateToPose, self.action_name, callback_group=self._cb_group)
        self.get_logger().info(f'Waiting for action server "{self.action_name}"...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('NavigateToPose action server not available yet')

        # subscribers
        self._path_sub = self.create_subscription(Path, '/goal_plan', self._path_cb, 10)
        self._pose_sub = self.create_subscription(PoseStamped, '/goal_plan_pose', self._pose_cb, 10)

        # publishers for visualization
        self._waypoints_pub = self.create_publisher(Path, 'patrol/waypoints', 10)
        self._next_goal_pub = self.create_publisher(PoseStamped, 'patrol/next_goal', 10)
        self._marker_pub = self.create_publisher(MarkerArray, 'patrol/markers', 10)
        # debug publisher for external inspection of plan (nav_msgs/Path)
        self._debug_pub = self.create_publisher(Path, '/debug_pose_Plan', 10)

        # subscriber to trigger save
        self._save_sub = self.create_subscription(Bool, '/save_point', self._save_cb, 10)
        # subscriber to manually start/stop
        self._start_sub = self.create_subscription(Bool, '/start_patrol', self._start_cb, 10)

        # clear service and topic to clear waypoints
        self._clear_service = self.create_service(Empty, 'clear_waypoints', self._clear_service_cb)
        self._clear_sub = self.create_subscription(Bool, '/clear_patrol', self._clear_sub_cb, 10)

        # control
        self._stop_flag = False
        self._patrol_thread = None
        self._current_waypoints = list(self.waypoints)
        self._lock = threading.Lock()
        # action lock to avoid concurrent send_goal_async usage
        self._action_lock = threading.Lock()
        # failure counters per waypoint index to avoid infinite retries
        self._fail_counts = {}
        self._max_fail_before_skip = 3

        # async patrol state
        self._goal_in_flight = False
        self._current_goal_handle = None
        self._current_idx = 0
        self._last_sent_idx = None
        self._last_start_cmd = None

        # create save dir path under package share
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('pb2025_nav2_patrol')
        self._save_root = os.path.join(pkg_share, 'save')
        os.makedirs(self._save_root, exist_ok=True)

        if self.auto_start and self._current_waypoints:
            self.start_patrol()

    def _normalize_waypoints(self, wps):
        if not wps:
            return []
        
        # If it's a list of strings like ["x,y,yaw", ...], parse them
        if isinstance(wps, list) and len(wps) > 0 and isinstance(wps[0], str):
            new = []
            for item in wps:
                try:
                    parts = item.split(',')
                    if len(parts) >= 2:
                        x = float(parts[0])
                        y = float(parts[1])
                        yaw = float(parts[2]) if len(parts) >= 3 else 0.0
                        new.append({'x': x, 'y': y, 'yaw': yaw})
                except Exception as e:
                    self.get_logger().warn(f'Failed to parse waypoint string "{item}": {e}')
            return new

        # If already list of dicts
        if isinstance(wps, list) and len(wps) > 0 and isinstance(wps[0], dict):
            return wps
        # If list of PoseStamped-like maps {x:, y:, yaw:}
        new = []
        for item in wps:
            if isinstance(item, dict) and {'x', 'y'}.issubset(item.keys()):
                new.append({'x': float(item.get('x', 0.0)), 'y': float(item.get('y', 0.0)), 'yaw': float(item.get('yaw', 0.0))})
        return new

    def _path_cb(self, msg: Path):
        # convert Path into waypoints (use pose.position.x/y and orientation yaw if available)
        wps = []
        for ps in msg.poses:
            p = ps.pose.position
            o = ps.pose.orientation
            yaw = self._quat_to_yaw(o)
            wps.append({'x': float(p.x), 'y': float(p.y), 'yaw': float(yaw)})
        with self._lock:
            # replace current waypoints with the received plan
            self._current_waypoints = wps
            self._current_idx = 0
        self.get_logger().info(f'Received /goal_plan with {len(wps)} points. Starting patrol.')

        # publish visualization of received waypoints
        self._publish_waypoints(wps)
        self._publish_markers(wps)
        # publish debug path
        try:
            self._debug_pub.publish(msg)
        except Exception:
            pass

        # start or continue patrol
        self.start_patrol()

    def _pose_cb(self, msg: PoseStamped):
        # append the received single pose to the patrol list (instead of replacing)
        p = msg.pose.position
        yaw = self._quat_to_yaw(msg.pose.orientation)
        new_wp = {'x': float(p.x), 'y': float(p.y), 'yaw': float(yaw)}
        with self._lock:
            # append to current waypoints
            self._current_waypoints.append(new_wp)
            wps_copy = list(self._current_waypoints)
        self.get_logger().info('Received single goal on /goal_plan_pose. Appended to patrol list.')

        # publish visualization for updated list
        self._publish_waypoints(wps_copy)
        self._publish_markers(wps_copy)
        # publish debug path built from current waypoints
        try:
            path = Path()
            path.header.frame_id = self.frame_id
            path.header.stamp = self.get_clock().now().to_msg()
            for wp in wps_copy:
                ps = PoseStamped()
                ps.header.frame_id = self.frame_id
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose.position.x = float(wp.get('x', 0.0))
                ps.pose.position.y = float(wp.get('y', 0.0))
                qx, qy, qz, qw = yaw_to_quaternion(float(wp.get('yaw', 0.0)))
                ps.pose.orientation.x = qx
                ps.pose.orientation.y = qy
                ps.pose.orientation.z = qz
                ps.pose.orientation.w = qw
                path.poses.append(ps)
            self._debug_pub.publish(path)
        except Exception:
            pass

        # ensure patrol is running based on auto_start policy
        if self.auto_start:
            self.start_patrol()
        else:
            self.get_logger().info('Waypoint appended. auto_start is false, waiting for /start_patrol to begin.')

    def _start_cb(self, msg: Bool):
        cmd = bool(msg.data)
        # Ignore repeated same command to avoid log flooding and redundant cancel/start calls.
        if self._last_start_cmd is not None and cmd == self._last_start_cmd:
            return
        self._last_start_cmd = cmd

        if cmd:
            self.get_logger().info('Manual start triggered via /start_patrol')
            self.start_patrol()
        else:
            self.get_logger().info('Manual stop triggered via /start_patrol')
            self.stop_patrol()

    def _quat_to_yaw(self, o):
        # assume normalized quaternion
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def start_patrol(self):
        # start async patrol: if no goal in flight, send next
        with self._lock:
            if not self._current_waypoints:
                self.get_logger().info('No waypoints to start patrol')
                return
        self._stop_flag = False
        # attempt to send next if none in flight
        self._maybe_send_next()

    def stop_patrol(self):
        # stop and cancel current goal if any
        self._stop_flag = True
        if self._current_goal_handle is not None:
            try:
                cancel_future = self._current_goal_handle.cancel_goal_async()
                # do not block; cancellation handled by server
            except Exception:
                pass
        self._goal_in_flight = False

    def _maybe_send_next(self):
        # send next goal if none in flight
        with self._lock:
            if self._stop_flag or self._goal_in_flight:
                return
            wps = list(self._current_waypoints)
            if not wps:
                return
            idx = self._current_idx % len(wps)
            wp = wps[idx]
        x = wp.get('x', 0.0)
        y = wp.get('y', 0.0)
        yaw = wp.get('yaw', 0.0)

        # publish next goal and markers
        next_pose = PoseStamped()
        next_pose.header.frame_id = self.frame_id
        next_pose.header.stamp = self.get_clock().now().to_msg()
        next_pose.pose.position.x = float(x)
        next_pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        next_pose.pose.orientation.x = qx
        next_pose.pose.orientation.y = qy
        next_pose.pose.orientation.z = qz
        next_pose.pose.orientation.w = qw
        try:
            self._next_goal_pub.publish(next_pose)
            self._publish_waypoints(wps)
            self._publish_markers(wps, highlight_idx=idx)
        except Exception:
            pass

        # send asynchronously
        with self._action_lock:
            try:
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = next_pose
                send_future = self._client.send_goal_async(goal_msg)
                send_future.add_done_callback(self._on_goal_response)
                self._goal_in_flight = True
                self._last_sent_idx = idx
            except Exception as e:
                self.get_logger().warn(f'Exception when sending goal async: {e}')
                self._goal_in_flight = False

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().warn(f'Goal response exception: {e}')
            self._goal_in_flight = False
            # treat as failure for this index
            self._handle_send_failure(self._last_sent_idx)
            return
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by action server')
            self._goal_in_flight = False
            self._handle_send_failure(self._last_sent_idx)
            return
        # accepted, request result
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'Exception getting goal result: {e}')
            result = None
        status = getattr(result, 'status', None) if result is not None else None
        idx = self._last_sent_idx
        self._goal_in_flight = False
        self._current_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal idx={idx} succeeded')
            # reset fail count
            self._fail_counts.pop(idx, None)
            # advance index
            with self._lock:
                self._current_idx = (self._current_idx + 1) % max(1, len(self._current_waypoints))
            # continue
            if not self._stop_flag:
                if self.wait_at_point > 0.0:
                    self.get_logger().info(f'Waiting for {self.wait_at_point} seconds before next goal...')
                    threading.Timer(self.wait_at_point, self._maybe_send_next).start()
                else:
                    self._maybe_send_next()
            return
        else:
            self.get_logger().warn(f'Goal idx={idx} failed status={status}')
            self._handle_send_failure(idx)

    def _handle_send_failure(self, idx):
        # increment fail count and decide retry or skip
        cnt = self._fail_counts.get(idx, 0) + 1
        self._fail_counts[idx] = cnt
        if cnt >= self._max_fail_before_skip:
            self.get_logger().warn(f'Waypoint idx={idx} failed {cnt} times, skipping')
            # reset count and advance index
            self._fail_counts.pop(idx, None)
            with self._lock:
                if self._current_waypoints:
                    self._current_idx = (self._current_idx + 1) % len(self._current_waypoints)
            # try next
            if not self._stop_flag:
                self._maybe_send_next()
            return
        if self.retry_on_fail:
            self.get_logger().info('Retrying failed goal immediately')
            # try again
            if not self._stop_flag:
                self._maybe_send_next()
            return
        # if not retry, skip
        with self._lock:
            if self._current_waypoints:
                self._current_idx = (self._current_idx + 1) % len(self._current_waypoints)
        if not self._stop_flag:
            self._maybe_send_next()

    def _publish_waypoints(self, wps):
        # publish nav_msgs/Path representing all waypoints
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for wp in wps:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(wp.get('x', 0.0))
            ps.pose.position.y = float(wp.get('y', 0.0))
            qx, qy, qz, qw = yaw_to_quaternion(float(wp.get('yaw', 0.0)))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        self._waypoints_pub.publish(path)

    def _publish_markers(self, wps, highlight_idx=None):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, wp in enumerate(wps):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = 'patrol_waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wp.get('x', 0.0))
            m.pose.position.y = float(wp.get('y', 0.0))
            m.pose.position.z = 0.05
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            if highlight_idx is not None and i == highlight_idx:
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
            else:
                m.color.r = 0.0
                m.color.g = 0.0
                m.color.b = 1.0
                m.color.a = 0.8
            ma.markers.append(m)
        # add a connecting line strip marker to show sequence
        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = now
        line.ns = 'patrol_lines'
        line.id = 1000
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.r = 1.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 0.6
        for wp in wps:
            p = Point()
            p.x = float(wp.get('x', 0.0))
            p.y = float(wp.get('y', 0.0))
            p.z = 0.05
            line.points.append(p)
        ma.markers.append(line)
        self._marker_pub.publish(ma)

    def _save_cb(self, msg: Bool):
        if not msg.data:
            return
        # snapshot current waypoints and write to yaml with timestamp
        with self._lock:
            wps = list(self._current_waypoints)
        if not wps:
            self.get_logger().warn('No waypoints to save')
            return
        ts = time.strftime('%Y%m%d_%H%M%S')
        save_dir = os.path.join(self._save_root, ts)
        os.makedirs(save_dir, exist_ok=True)
        # save waypoints yaml
        way_file = os.path.join(save_dir, 'waypoints.yaml')
        try:
            with open(way_file, 'w') as f:
                yaml.dump({'waypoints': wps}, f)
            self.get_logger().info(f'Waypoints saved to {way_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')
            return
        # try to save map via nav2 map_saver
        map_file_base = os.path.join(save_dir, 'map')
        try:
            # map_saver_cli from nav2_map_server, will produce <map>.yaml and <map>.pgm
            cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_file_base]
            self.get_logger().info(f'Running map_saver: {" ".join(cmd)}')
            subprocess.run(cmd, check=True)
            self.get_logger().info(f'Map saved to {map_file_base}.yaml/.pgm')
        except Exception as e:
            self.get_logger().error(f'Failed to save map via map_saver_cli: {e}')

    def _do_clear(self):
        # stop patrol thread and clear waypoints
        self.get_logger().info('Clearing patrol waypoints')
        # stop thread
        self._stop_flag = True
        if self._patrol_thread and self._patrol_thread.is_alive():
            try:
                self._patrol_thread.join(timeout=1.0)
            except Exception:
                pass
        self._patrol_thread = None
        with self._lock:
            self._current_waypoints = []
        # publish empty visuals
        try:
            self._publish_waypoints([])
            self._publish_markers([])
            # publish empty debug path
            path = Path()
            path.header.frame_id = self.frame_id
            path.header.stamp = self.get_clock().now().to_msg()
            self._debug_pub.publish(path)
        except Exception:
            pass
        self.get_logger().info('Patrol waypoints cleared')

    def _clear_service_cb(self, request, response):
        self._do_clear()
        return Empty.Response()

    def _clear_sub_cb(self, msg: Bool):
        if msg.data:
            self._do_clear()

    def destroy_node(self):
        self._stop_flag = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
