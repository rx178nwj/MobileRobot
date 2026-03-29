#!/usr/bin/env python3
"""
Autonomous Indoor Mapping Node
Ubuntu Server Side — ROS 2 Jazzy

Drives the robot through unknown space to build a complete SLAM map.
Uses LLM vision to choose the best frontier target in real time.

Strategy:
  1. Detect frontier cells — boundary between free and unknown space
  2. Ask LLM (with camera image + frontier list) which frontier to visit
  3. Navigate there via Nav2 NavigateToPose action
  4. Repeat until no frontiers remain or /mapping/stop is received

Subscribed topics:
  /map                          (nav_msgs/OccupancyGrid, TRANSIENT_LOCAL)
  /odom                         (nav_msgs/Odometry)
  /camera/image_raw/compressed  (sensor_msgs/CompressedImage)
  /mapping/stop                 (std_msgs/Empty)

Published topics:
  /mapping/status               (std_msgs/String)  human-readable log

Action client:
  navigate_to_pose              (nav2_msgs/action/NavigateToPose)

Parameters:
  llm_base_url        URL of OpenAI-compatible API server
  llm_model           model name (e.g. openai/gpt-oss-20b)
  openai_api_key      API key (use "local" for LM Studio)
  exploration_timeout Total exploration time limit in seconds (0=unlimited)
  nav_timeout         Per-waypoint navigation timeout in seconds
  stuck_timeout       If no progress for this many seconds, pick new target
  max_frontier_dist   Only consider frontiers within this radius (m, 0=all)
  min_frontier_size   Ignore frontier clusters smaller than this (cells)
  llm_interval        Minimum seconds between LLM calls

Usage:
  ros2 run mobile_robot_server autonomous_mapping
  ros2 topic pub /mapping/stop std_msgs/msg/Empty {} --once
"""

import base64
import json
import math
import re
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty, String
from nav2_msgs.action import NavigateToPose

try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False


# ---------------------------------------------------------------------------
# Frontier detection helpers
# ---------------------------------------------------------------------------

class FrontierDetector:
    """Detect and cluster frontier cells in an OccupancyGrid."""

    # OccupancyGrid values
    FREE     = 0
    UNKNOWN  = -1
    OCCUPIED = 100

    @staticmethod
    def find_frontiers(
        grid: OccupancyGrid,
        min_cluster_size: int = 5,
    ) -> List[Tuple[float, float, int]]:
        """
        Return list of (world_x, world_y, cluster_size) for each frontier cluster.

        A frontier cell: value == FREE and at least one 4-connected neighbour is UNKNOWN.
        Clusters are formed by 8-connected components of frontier cells.
        """
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        data = np.array(grid.data, dtype=np.int8).reshape((h, w))

        # Frontier mask: free cell adjacent to unknown
        free    = (data == FrontierDetector.FREE)
        unknown = (data == FrontierDetector.UNKNOWN)

        # Dilate unknown by 1 pixel (4-connected) to find free cells next to unknown
        kernel_4 = np.array([[0, 1, 0],
                              [1, 1, 1],
                              [0, 1, 0]], dtype=np.uint8)
        unknown_u8 = unknown.astype(np.uint8)
        unknown_dilated = cv2.dilate(unknown_u8, kernel_4, iterations=1).astype(bool)

        frontier_mask = (free & unknown_dilated).astype(np.uint8)

        # Cluster frontier cells with 8-connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            frontier_mask, connectivity=8
        )

        result = []
        for label in range(1, num_labels):  # 0 = background
            size = int(stats[label, cv2.CC_STAT_AREA])
            if size < min_cluster_size:
                continue
            cx_px = centroids[label][0]
            cy_px = centroids[label][1]
            # Convert pixel centre → world coordinates
            world_x = ox + (cx_px + 0.5) * res
            world_y = oy + (cy_px + 0.5) * res
            result.append((world_x, world_y, size))

        return result

    @staticmethod
    def coverage(grid: OccupancyGrid) -> float:
        """Fraction of cells that are known (free or occupied)."""
        data = np.array(grid.data, dtype=np.int8)
        total = len(data)
        if total == 0:
            return 0.0
        known = int(np.sum(data >= 0))
        return known / total


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class AutonomousMapper(Node):

    def __init__(self):
        super().__init__('autonomous_mapper')

        # Parameters
        self.declare_parameter('llm_base_url',
                               'http://Nadia.local:1234/v1')
        self.declare_parameter('llm_model', 'openai/gpt-oss-20b')
        self.declare_parameter('openai_api_key', 'local')
        self.declare_parameter('exploration_timeout', 0.0)   # 0 = unlimited
        self.declare_parameter('nav_timeout',          60.0)
        self.declare_parameter('stuck_timeout',        60.0)
        self.declare_parameter('max_frontier_dist',     0.0)  # 0 = all
        self.declare_parameter('min_frontier_size',    10)
        self.declare_parameter('llm_interval',          5.0)

        self._llm_base_url   = self.get_parameter('llm_base_url').value
        self._llm_model      = self.get_parameter('llm_model').value
        self._api_key        = self.get_parameter('openai_api_key').value
        self._explore_limit  = self.get_parameter('exploration_timeout').value
        self._nav_timeout    = self.get_parameter('nav_timeout').value
        self._stuck_timeout  = self.get_parameter('stuck_timeout').value
        self._max_dist       = self.get_parameter('max_frontier_dist').value
        self._min_cluster    = self.get_parameter('min_frontier_size').value
        self._llm_interval   = self.get_parameter('llm_interval').value

        # LLM client
        self._llm_client: Optional[OpenAI] = None
        if OPENAI_AVAILABLE:
            self._llm_client = OpenAI(
                api_key=self._api_key,
                base_url=self._llm_base_url,
            )
            self.get_logger().info(
                f'LLM client: {self._llm_base_url}  model={self._llm_model}')
        else:
            self.get_logger().warn(
                'openai package not found — LLM disabled, using closest-frontier fallback')

        # State
        self._lock         = threading.Lock()
        self._latest_map:   Optional[OccupancyGrid] = None
        self._latest_image: Optional[bytes] = None   # raw JPEG bytes
        self._pose:         Tuple[float, float, float] = (0.0, 0.0, 0.0)  # x, y, yaw
        self._stop_flag     = False
        self._last_llm_time = 0.0

        # QoS for /map (TRANSIENT_LOCAL so we get the latest even if subscribed late)
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Subscribers
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self._img_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self._img_cb, 10)
        self._stop_sub = self.create_subscription(
            Empty, '/mapping/stop', self._stop_cb, 10)

        # Publisher
        self._status_pub = self.create_publisher(String, '/mapping/status', 10)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Start exploration loop in background thread
        threading.Thread(target=self._explore_loop, daemon=True).start()
        self.get_logger().info('AutonomousMapper started')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        with self._lock:
            self._latest_map = msg

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        with self._lock:
            self._pose = (p.x, p.y, yaw)

    def _img_cb(self, msg: CompressedImage):
        with self._lock:
            self._latest_image = bytes(msg.data)

    def _stop_cb(self, _msg: Empty):
        self.get_logger().info('/mapping/stop received — stopping exploration')
        with self._lock:
            self._stop_flag = True

    # -----------------------------------------------------------------------
    # Exploration loop
    # -----------------------------------------------------------------------

    def _explore_loop(self):
        self._publish_status('Waiting for map and Nav2...')

        # Wait for first map
        while rclpy.ok():
            with self._lock:
                has_map = self._latest_map is not None
                stop    = self._stop_flag
            if stop:
                return
            if has_map:
                break
            time.sleep(1.0)

        # Wait for Nav2 action server
        self._publish_status('Waiting for navigate_to_pose action server...')
        if not self._nav_client.wait_for_server(timeout_sec=30.0):
            self._publish_status('ERROR: navigate_to_pose action server not available')
            self.get_logger().error('navigate_to_pose not available after 30s')
            return

        self._publish_status('Exploration started')
        start_time = time.monotonic()

        while rclpy.ok():
            with self._lock:
                stop = self._stop_flag
                grid = self._latest_map

            if stop:
                self._publish_status('Exploration stopped by request')
                break

            # Check time limit
            elapsed = time.monotonic() - start_time
            if self._explore_limit > 0.0 and elapsed > self._explore_limit:
                self._publish_status(
                    f'Exploration time limit reached ({self._explore_limit:.0f}s)')
                break

            # Get current state
            with self._lock:
                pose  = self._pose
                image = self._latest_image

            # Detect frontiers
            frontiers = FrontierDetector.find_frontiers(
                grid, min_cluster_size=self._min_cluster)

            if not frontiers:
                cov = FrontierDetector.coverage(grid)
                self._publish_status(
                    f'No frontiers remaining — exploration complete! '
                    f'Coverage: {cov*100:.1f}%')
                break

            # Filter by max distance
            if self._max_dist > 0.0:
                frontiers = [
                    f for f in frontiers
                    if math.hypot(f[0] - pose[0], f[1] - pose[1]) <= self._max_dist
                ]
                if not frontiers:
                    self._publish_status('All frontiers beyond max_frontier_dist — done')
                    break

            # Coverage stats for logging
            cov = FrontierDetector.coverage(grid)

            # Choose target frontier
            target = self._choose_frontier(frontiers, pose, image, cov)

            if target is None:
                self.get_logger().warn('Could not choose frontier, retrying in 3s')
                time.sleep(3.0)
                continue

            tx, ty = target
            dist = math.hypot(tx - pose[0], ty - pose[1])
            self._publish_status(
                f'Navigating to frontier ({tx:.2f}, {ty:.2f})  '
                f'dist={dist:.2f}m  coverage={cov*100:.1f}%  '
                f'frontiers={len(frontiers)}')

            # Navigate
            success = self._navigate_to(tx, ty, 0.0)
            if not success:
                self.get_logger().warn(
                    f'Navigation to ({tx:.2f}, {ty:.2f}) failed — trying next frontier')
            time.sleep(0.5)  # brief pause before re-scanning

        self._publish_status('Exploration loop exited')

    # -----------------------------------------------------------------------
    # Frontier selection
    # -----------------------------------------------------------------------

    def _choose_frontier(
        self,
        frontiers: List[Tuple[float, float, int]],
        pose: Tuple[float, float, float],
        image: Optional[bytes],
        coverage: float,
    ) -> Optional[Tuple[float, float]]:
        """
        Ask LLM which frontier to visit, or fall back to closest large frontier.
        Returns (x, y) in world frame.
        """
        # Throttle LLM calls
        now = time.monotonic()
        use_llm = (
            self._llm_client is not None
            and image is not None
            and (now - self._last_llm_time) >= self._llm_interval
        )

        if use_llm:
            result = self._ask_llm(frontiers, pose, image, coverage)
            if result is not None:
                self._last_llm_time = now
                return result
            self.get_logger().warn('LLM returned no decision — using fallback')

        # Fallback: pick largest frontier within closest 3 candidates
        px, py, _ = pose
        sorted_by_dist = sorted(
            frontiers,
            key=lambda f: math.hypot(f[0] - px, f[1] - py)
        )
        candidates = sorted_by_dist[:5]
        # Among closest 5, pick the largest cluster (more unexplored area)
        best = max(candidates, key=lambda f: f[2])
        return (best[0], best[1])

    # -----------------------------------------------------------------------
    # LLM interaction
    # -----------------------------------------------------------------------

    _SYSTEM_PROMPT = (
        'You are the navigation planner for an autonomous indoor mapping robot. '
        'The robot uses frontier-based exploration: it navigates to boundaries '
        'between free space and unknown space to reveal the map. '
        'You receive: a camera image, the robot\'s current position and heading, '
        'a numbered list of frontier candidates (world coordinates + size), '
        'and the current map coverage percentage. '
        'Choose the frontier index that will most efficiently expand map coverage, '
        'considering: large frontier clusters (more unknown area), '
        'reachability (avoid areas just behind obstacles visible in camera), '
        'and exploration diversity (avoid re-visiting recently explored directions). '
        'Reply with ONLY valid JSON, no markdown, no explanation:\n'
        '{"frontier_index": <integer>, "reasoning": "<one sentence>", '
        '"continue_exploring": true|false}'
    )

    def _ask_llm(
        self,
        frontiers: List[Tuple[float, float, int]],
        pose: Tuple[float, float, float],
        image: bytes,
        coverage: float,
    ) -> Optional[Tuple[float, float]]:
        """Query LLM for best frontier. Returns (x, y) or None on failure."""
        px, py, yaw_rad = pose
        yaw_deg = math.degrees(yaw_rad)

        # Build frontier list text
        frontier_lines = []
        for i, (fx, fy, sz) in enumerate(frontiers):
            dist = math.hypot(fx - px, fy - py)
            angle = math.degrees(math.atan2(fy - py, fx - px) - yaw_rad)
            # Normalise angle to [-180, 180]
            angle = (angle + 180) % 360 - 180
            frontier_lines.append(
                f'  [{i}] x={fx:.2f} y={fy:.2f}  '
                f'dist={dist:.2f}m  angle={angle:.0f}°  size={sz}cells'
            )
        frontier_text = '\n'.join(frontier_lines)

        user_text = (
            f'Robot pose: x={px:.2f} y={py:.2f} heading={yaw_deg:.0f}°\n'
            f'Map coverage: {coverage*100:.1f}%\n'
            f'Frontier candidates:\n{frontier_text}\n\n'
            'Which frontier should the robot visit next?'
        )

        # Encode image as base64 JPEG
        b64 = base64.b64encode(image).decode('utf-8')
        user_content = [
            {
                'type': 'image_url',
                'image_url': {
                    'url': f'data:image/jpeg;base64,{b64}',
                    'detail': 'low',
                },
            },
            {'type': 'text', 'text': user_text},
        ]

        try:
            response = self._llm_client.chat.completions.create(
                model=self._llm_model,
                messages=[
                    {'role': 'system', 'content': self._SYSTEM_PROMPT},
                    {'role': 'user',   'content': user_content},
                ],
                max_tokens=512,
                temperature=0.1,
            )
            msg = response.choices[0].message
            raw = (msg.content or '').strip()
            if not raw:
                raw = (getattr(msg, 'reasoning_content', None) or '').strip()

            self.get_logger().info(f'LLM raw: {raw[:300]}')

            # Strip <think> blocks
            if '<think>' in raw:
                raw = re.sub(r'<think>.*?</think>', '', raw, flags=re.DOTALL).strip()

            # Strip markdown fences
            if raw.startswith('```'):
                raw = raw.split('```')[1]
                if raw.startswith('json'):
                    raw = raw[4:]

            # Extract first JSON object
            m = re.search(r'\{.*\}', raw, re.DOTALL)
            if not m:
                self.get_logger().error(f'No JSON in LLM response: {raw!r}')
                return None

            data = json.loads(m.group())
            idx = int(data.get('frontier_index', -1))
            reasoning = data.get('reasoning', '')
            continue_flag = bool(data.get('continue_exploring', True))

            self.get_logger().info(
                f'LLM chose frontier [{idx}]: {reasoning}  continue={continue_flag}')

            if not continue_flag:
                self.get_logger().info('LLM says exploration complete')
                with self._lock:
                    self._stop_flag = True
                return None

            if idx < 0 or idx >= len(frontiers):
                self.get_logger().error(
                    f'LLM returned invalid frontier index {idx} '
                    f'(have {len(frontiers)} frontiers)')
                return None

            self._publish_status(f'LLM: [{idx}] {reasoning}')
            return (frontiers[idx][0], frontiers[idx][1])

        except Exception as e:
            self.get_logger().error(f'LLM call failed: {e}')
            return None

    # -----------------------------------------------------------------------
    # Navigation
    # -----------------------------------------------------------------------

    def _navigate_to(self, x: float, y: float, yaw: float) -> bool:
        """
        Send NavigateToPose goal and block until done or timeout.
        Returns True if goal succeeded.
        """
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        # Convert yaw to quaternion
        goal.pose.pose.orientation = _yaw_to_quat(yaw)

        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f})')
        send_future = self._nav_client.send_goal_async(goal)

        # Wait for goal acceptance
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(0.1)  # main thread executor already spinning
            if send_future.done():
                break

        if not send_future.done():
            self.get_logger().error('Goal send timed out')
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + self._nav_timeout
        last_pose = None
        last_move_time = time.monotonic()

        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                stop = self._stop_flag
                curr_pose = self._pose

            if stop:
                goal_handle.cancel_goal_async()
                return False

            # Stuck detection
            if last_pose is not None:
                moved = math.hypot(
                    curr_pose[0] - last_pose[0],
                    curr_pose[1] - last_pose[1]
                )
                if moved > 0.05:
                    last_move_time = time.monotonic()
            last_pose = curr_pose

            if time.monotonic() - last_move_time > self._stuck_timeout:
                self.get_logger().warn(
                    f'Stuck for {self._stuck_timeout:.0f}s — cancelling goal')
                goal_handle.cancel_goal_async()
                return False

            time.sleep(0.2)  # main thread executor already spinning

            if result_future.done():
                status = result_future.result().status
                success = (status == GoalStatus.STATUS_SUCCEEDED)
                self.get_logger().info(
                    f'Goal finished with status {status} '
                    f'({"success" if success else "failed"})')
                return success

        # Timeout reached
        self.get_logger().warn(
            f'Navigation timeout ({self._nav_timeout:.0f}s) — cancelling')
        goal_handle.cancel_goal_async()
        return False

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(f'[mapping] {text}')


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMapper()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
