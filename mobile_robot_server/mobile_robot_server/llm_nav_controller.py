#!/usr/bin/env python3
"""
LLM Navigation Controller

AI pipeline:
  Edge camera frame
    └→ YOLO (server GPU, yolov8s.pt)  — object detection
         └→ scene description (text)
              └→ Ollama qwen3.5:9b (localhost:11434)  — navigation decision
                   └→ ROS 2 /cmd_vel or /goal_pose

Modes (set via /llm_command topic, JSON payload):
  navigate  - Move toward a text-described goal, avoiding obstacles
              {"mode": "navigate", "command": "go to the door"}
  track     - Detect and follow a person in the camera frame
              {"mode": "track"}
  explore   - Autonomously explore the room, building the SLAM map
              {"mode": "explore"}
  idle      - Stop all motion
              {"mode": "idle"}

Published topics:
  /cmd_vel      (geometry_msgs/Twist)  — direct velocity commands
  /goal_pose    (geometry_msgs/PoseStamped) — Nav2 goal (navigate mode)
  /llm_status   (std_msgs/String)      — human-readable status log

Subscribed topics:
  /camera/image_raw  (sensor_msgs/Image)
  /llm_command       (std_msgs/String)
  /odom              (nav_msgs/Odometry)

Environment variables:
  LLM_BASE_URL   — Ollama endpoint (default: http://localhost:11434/v1)
  LLM_MODEL      — model name (default: qwen3.5:9b)
  YOLO_MODEL     — YOLO weights (default: yolov8s.pt)
"""

import base64
import json
import os
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except Exception:
    YOLO_AVAILABLE = False


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MODE_NAVIGATE = 'navigate'
MODE_TRACK = 'track'
MODE_EXPLORE = 'explore'
MODE_IDLE = 'idle'

_SYSTEM_NAVIGATE = (
    'You are controlling a differential-drive mobile robot. '
    'You receive a scene description and a navigation goal. '
    'Determine the single best action to take right now. '
    'Reply with ONLY valid JSON, no markdown, no explanation: '
    '{"action": "move_forward"|"turn_left"|"turn_right"|"stop"|"arrived", '
    '"reason": "<one sentence>"}'
)

_SYSTEM_TRACK = (
    'You are controlling a differential-drive mobile robot that must follow a person. '
    'Analyze the scene description. '
    'If a person is visible: determine whether they are left/center/right and how far. '
    'If no person: search by turning slowly. '
    'Reply with ONLY valid JSON, no markdown, no explanation: '
    '{"action": "move_forward"|"turn_left"|"turn_right"|"stop"|"search", '
    '"person_detected": true|false, '
    '"person_position": "left"|"center"|"right"|"none", '
    '"reason": "<one sentence>"}'
)

_SYSTEM_EXPLORE = (
    'You are controlling a differential-drive mobile robot exploring an unknown room. '
    'You receive a text description of the scene. '
    'Choose the safest direction to move. '
    'Reply with ONLY valid JSON, no markdown, no explanation: '
    '{"action": "move_forward"|"turn_left"|"turn_right"|"stop", '
    '"observation": "<one sentence>"}'
)


class LLMNavController(Node):

    def __init__(self):
        super().__init__('llm_nav_controller')

        # ---- Parameters ----
        self.declare_parameter('openai_api_key', os.environ.get('OPENAI_API_KEY', 'ollama'))
        self.declare_parameter('llm_base_url', os.environ.get('LLM_BASE_URL', 'http://localhost:11434/v1'))
        self.declare_parameter('model', os.environ.get('LLM_MODEL', 'qwen3.5:9b'))
        self.declare_parameter('llm_interval', 2.0)   # seconds between LLM calls
        self.declare_parameter('linear_speed', 0.25)  # m/s forward
        self.declare_parameter('angular_speed', 0.5)  # rad/s turning
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('max_image_width', 480)
        self.declare_parameter('vision_enabled', False)         # Ollama text-only; use YOLO instead
        self.declare_parameter('use_object_detection', True)    # YOLO on server GPU
        self.declare_parameter('yolo_model', os.environ.get('YOLO_MODEL', 'yolov8s.pt'))
        self.declare_parameter('yolo_conf', 0.3)

        api_key = self.get_parameter('openai_api_key').value
        base_url = self.get_parameter('llm_base_url').value
        self.model = self.get_parameter('model').value
        self.llm_interval = self.get_parameter('llm_interval').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.max_image_width = self.get_parameter('max_image_width').value
        self.vision_enabled = self.get_parameter('vision_enabled').value
        self.use_object_detection = self.get_parameter('use_object_detection').value
        self.yolo_conf = self.get_parameter('yolo_conf').value

        # YOLO setup
        self.yolo = None
        if self.use_object_detection:
            if YOLO_AVAILABLE:
                yolo_model = self.get_parameter('yolo_model').value
                self.yolo = YOLO(yolo_model)
                self.get_logger().info(f'YOLO object detection enabled (model: {yolo_model})')
            else:
                self.get_logger().warn(
                    'use_object_detection=True but ultralytics not installed; '
                    'falling back to edge-based scene analysis'
                )

        if not OPENAI_AVAILABLE:
            self.get_logger().error('openai package not installed — run: pip install openai')
        else:
            client_kwargs = {'api_key': api_key or 'local'}
            if base_url:
                client_kwargs['base_url'] = base_url
            self.llm_client = OpenAI(**client_kwargs)
            self.get_logger().info(
                f'LLM client ready (model: {self.model}, base_url: {base_url or "openai default"})'
            )

        self.bridge = CvBridge()

        # ---- State (protected by _lock) ----
        self._lock = threading.Lock()
        self._mode = MODE_IDLE
        self._command = ''
        self._latest_frame: np.ndarray | None = None
        self._last_llm_time = 0.0
        self._llm_busy = False
        self._current_twist = Twist()  # last action, republished at 4 Hz

        # ---- Subscribers ----
        self.create_subscription(Image, '/camera/image_raw', self._cb_image, 10)
        self.create_subscription(String, '/llm_command', self._cb_command, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)

        # ---- Publishers ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/llm_status', 10)

        # ---- Control timer (10 Hz) — LLM scheduling ----
        self.create_timer(0.1, self._control_tick)
        # ---- Republish timer (4 Hz) — keep motor service alive ----
        self.create_timer(0.25, self._republish_twist)

        self.get_logger().info('LLM Nav Controller started — waiting for /llm_command')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _cb_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._lock:
                self._latest_frame = frame
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}')

    def _cb_odom(self, msg: Odometry):
        # Reserved for future position-aware navigation logic
        pass

    def _cb_command(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in /llm_command: {msg.data!r}')
            return

        mode = data.get('mode', MODE_IDLE)
        command = data.get('command', '')

        with self._lock:
            self._mode = mode
            self._command = command
            self._last_llm_time = 0.0  # trigger LLM immediately on mode change

        self._publish_status(f'[mode={mode}] {command}')
        self.get_logger().info(f'Command received: mode={mode} command={command!r}')

        # If switching to idle, stop immediately
        if mode == MODE_IDLE:
            self._stop_robot()

    # -----------------------------------------------------------------------
    # Control loop
    # -----------------------------------------------------------------------

    def _control_tick(self):
        with self._lock:
            mode = self._mode
            frame = self._latest_frame
            command = self._command
            now = time.monotonic()
            elapsed = now - self._last_llm_time
            busy = self._llm_busy

        if mode == MODE_IDLE or busy:
            return
        if (self.vision_enabled or self.use_object_detection) and frame is None:
            return

        if elapsed < self.llm_interval:
            return

        with self._lock:
            self._last_llm_time = now
            self._llm_busy = True
            frame_copy = frame.copy() if frame is not None else None

        threading.Thread(
            target=self._llm_step,
            args=(mode, frame_copy, command),
            daemon=True
        ).start()

    # -----------------------------------------------------------------------
    # LLM interaction
    # -----------------------------------------------------------------------

    def _llm_step(self, mode: str, frame: np.ndarray, command: str):
        try:
            if mode == MODE_NAVIGATE:
                self._step_navigate(frame, command)
            elif mode == MODE_TRACK:
                self._step_track(frame)
            elif mode == MODE_EXPLORE:
                self._step_explore(frame)
        except Exception as e:
            self.get_logger().error(f'LLM step error: {e}')
        finally:
            with self._lock:
                self._llm_busy = False

    def _extract_scene_description(self, frame: np.ndarray) -> str:
        """Detect objects in frame and return text description for the LLM."""
        h, w = frame.shape[:2]

        if self.yolo is not None:
            results = self.yolo(frame, verbose=False, conf=self.yolo_conf)[0]
            if len(results.boxes) == 0:
                return 'No objects detected in the camera view.'
            items = []
            for box in results.boxes:
                cls_name = results.names[int(box.cls[0])]
                x_center = float(box.xywhn[0][0])
                y_center = float(box.xywhn[0][1])
                pos_h = 'left' if x_center < 0.33 else ('right' if x_center > 0.67 else 'center')
                pos_v = 'close' if y_center > 0.6 else 'far'
                items.append(f'{cls_name} ({pos_h}, {pos_v})')
            return 'Detected objects: ' + ', '.join(items) + '.'

        # Fallback: edge density analysis per region
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        zones = {
            'left':   cv2.Canny(gray[:, :w // 3],         50, 150).mean(),
            'center': cv2.Canny(gray[:, w // 3:2 * w // 3], 50, 150).mean(),
            'right':  cv2.Canny(gray[:, 2 * w // 3:],     50, 150).mean(),
        }
        thresh = 5.0
        parts = [
            'obstacle ahead' if zones['center'] > thresh else 'clear path ahead',
            'obstacle on left' if zones['left'] > thresh else 'open space on left',
            'obstacle on right' if zones['right'] > thresh else 'open space on right',
        ]
        return 'Scene analysis: ' + ', '.join(parts) + '.'

    def _call_vision(self, frame: np.ndarray | None, system: str, user: str) -> dict:
        """Send prompt (+ image or scene description) to LLM, return parsed JSON dict."""
        if self.vision_enabled and frame is not None:
            b64 = self._encode_frame(frame)
            user_content = [
                {
                    'type': 'image_url',
                    'image_url': {
                        'url': f'data:image/jpeg;base64,{b64}',
                        'detail': 'low',
                    },
                },
                {'type': 'text', 'text': user},
            ]
        elif self.use_object_detection and frame is not None:
            scene = self._extract_scene_description(frame)
            self.get_logger().debug(f'Scene: {scene}')
            user_content = f'/no_think {scene}\n\n{user}'
        else:
            user_content = f'/no_think {user}'

        response = self.llm_client.chat.completions.create(
            model=self.model,
            messages=[
                {'role': 'system', 'content': system},
                {'role': 'user', 'content': user_content},
            ],
            max_tokens=2000,
            temperature=0.1,
        )
        msg = response.choices[0].message
        raw = (msg.content or '').strip()
        # Qwen3 thinking mode: answer may be in reasoning_content when content is empty
        if not raw:
            raw = (getattr(msg, 'reasoning_content', None) or '').strip()
        self.get_logger().info(f'LLM raw: {raw[:300]}')
        # Strip <think>...</think> blocks
        if '<think>' in raw:
            import re
            raw = re.sub(r'<think>.*?</think>', '', raw, flags=re.DOTALL).strip()
        # Strip markdown fences if model adds them
        if raw.startswith('```'):
            raw = raw.split('```')[1]
            if raw.startswith('json'):
                raw = raw[4:]
        # Extract first JSON object if extra text surrounds it
        if raw and not raw.startswith('{'):
            import re
            m = re.search(r'\{.*\}', raw, re.DOTALL)
            if m:
                raw = m.group(0)
        return json.loads(raw)

    def _encode_frame(self, frame: np.ndarray) -> str:
        h, w = frame.shape[:2]
        if w > self.max_image_width:
            scale = self.max_image_width / w
            frame = cv2.resize(frame, (self.max_image_width, int(h * scale)))
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        return base64.b64encode(buf.tobytes()).decode('utf-8')

    # -----------------------------------------------------------------------
    # Mode handlers
    # -----------------------------------------------------------------------

    def _step_navigate(self, frame: np.ndarray, command: str):
        try:
            resp = self._call_vision(
                frame,
                _SYSTEM_NAVIGATE,
                f'Navigation goal: "{command}". What should I do next?',
            )
        except (json.JSONDecodeError, Exception) as e:
            self.get_logger().warn(f'Navigate LLM failed: {e}')
            return

        action = resp.get('action', 'stop')
        reason = resp.get('reason', '')
        self.get_logger().info(f'[navigate] {action} — {reason}')
        self._publish_status(f'[navigate] {action}: {reason}')
        self._execute_action(action)

    def _step_track(self, frame: np.ndarray):
        try:
            resp = self._call_vision(
                frame,
                _SYSTEM_TRACK,
                'Where is the person? How should I move to follow them?',
            )
        except (json.JSONDecodeError, Exception) as e:
            self.get_logger().warn(f'Track LLM failed: {e}')
            return

        action = resp.get('action', 'stop')
        detected = resp.get('person_detected', False)
        position = resp.get('person_position', 'none')
        reason = resp.get('reason', '')
        self.get_logger().info(
            f'[track] detected={detected} pos={position} action={action} — {reason}'
        )
        self._publish_status(
            f'[track] {"person @ " + position if detected else "searching"}: {reason}'
        )
        self._execute_action(action)

    def _step_explore(self, frame: np.ndarray):
        try:
            resp = self._call_vision(
                frame,
                _SYSTEM_EXPLORE,
                'What do you see? Where should I move to explore safely?',
            )
        except (json.JSONDecodeError, Exception) as e:
            self.get_logger().warn(f'Explore LLM failed: {e}')
            return

        action = resp.get('action', 'stop')
        observation = resp.get('observation', '')
        self.get_logger().info(f'[explore] {action} — {observation}')
        self._publish_status(f'[explore] {action}: {observation}')
        self._execute_action(action)

    # -----------------------------------------------------------------------
    # Motion primitives
    # -----------------------------------------------------------------------

    def _execute_action(self, action: str):
        twist = Twist()
        if action == 'move_forward':
            twist.linear.x = self.linear_speed
        elif action == 'turn_left':
            twist.angular.z = self.angular_speed
        elif action == 'turn_right':
            twist.angular.z = -self.angular_speed
        elif action == 'search':
            twist.angular.z = self.angular_speed * 0.5
        # 'stop' and 'arrived' → zero twist (robot stops)
        with self._lock:
            self._current_twist = twist
        self.cmd_vel_pub.publish(twist)

    def _republish_twist(self):
        """Republish last twist at 4 Hz to prevent motor safety timeout."""
        with self._lock:
            mode = self._mode
            twist = self._current_twist
        if mode != MODE_IDLE:
            self.cmd_vel_pub.publish(twist)

    def _stop_robot(self):
        with self._lock:
            self._current_twist = Twist()
        self.cmd_vel_pub.publish(Twist())

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = LLMNavController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
