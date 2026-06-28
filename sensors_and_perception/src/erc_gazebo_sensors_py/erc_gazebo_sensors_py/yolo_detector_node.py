import time
import math
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO

# ─────────────────────────────────────────────────────────────────
#  TUNABLE CONSTANTS
# ─────────────────────────────────────────────────────────────────
YOLO_MODEL          = 'yolov8n.pt'

# ── KEY FIX: lower confidence for Gazebo synthetic renders ────────
# Gazebo objects look nothing like real photos YOLO trained on.
# Real-world use: 0.50. Gazebo simulation: 0.20-0.30.
CONF_THRESHOLD      = 0.25

STOP_DISTANCE       = 0.70
APPROACH_SLOW       = 1.20

SCAN_ANGULAR_SPEED  = 0.30
FULL_ROTATION       = 2 * math.pi * 0.92
LOST_GRACE_FRAMES   = 8
REACQUIRE_MAX_ARC   = math.radians(70)

EXPLORE_LINEAR      = 0.30
EXPLORE_TURN_KP     = 1.4
EXPLORE_DRIVE_KP    = 1.0
EXPLORE_TURN_TOL    = 0.12
EXPLORE_SLOW_DIST   = 1.50
MAX_EXPLORE_DIST    = 6.0

OBSTACLE_FRONT      = 1.00
ROBOT_HALF_WIDTH    = 0.20
SAFETY_MARGIN       = 0.10
ROTATE_SAFE_RADIUS  = 0.35
MIN_OPEN_DIST       = 1.00
GRID_CELL           = 0.5
CANDIDATE_STEP_DEG  = 15
WINDOW_DEG          = 12

SEEN_MARK_MAX       = 3.0
SEEN_RAY_STEP_DEG   = 4
EXPLORE_HORIZON     = 2.2
OPENNESS_CAP        = 2.0
UNSEEN_BONUS        = 5.0

CENTER_TOL_PX       = 45
KP_ANGULAR          = 0.003
SPEED_FAST          = 0.40
SPEED_SLOW          = 0.18
DEPTH_PATCH         = 9
# Desired distance to maintain from the target
FOLLOW_DISTANCE = 1.20      # meters

# Small tolerance to prevent constant forward/backward oscillation
FOLLOW_TOLERANCE = 0.10

# Mission considered complete when robot reaches follow distance
STOP_DISTANCE = FOLLOW_DISTANCE
# ── DEBUG: print every YOLO detection to terminal ─────────────────
# Helps diagnose what the model actually sees in simulation.
DEBUG_DETECTIONS    = True
DEBUG_PRINT_EVERY   = 30   # print every N frames (not every frame, too noisy)


# ─────────────────────────────────────────────────────────────────
#  STATE MACHINE
# ─────────────────────────────────────────────────────────────────
class State:
    IDLE       = "IDLE"
    SCAN       = "SCAN"
    EXPLORE    = "EXPLORE"
    REACQUIRE  = "REACQUIRE"
    TRACKING   = "TRACKING"
    APPROACH   = "APPROACH"
    COMPLETE   = "COMPLETE"


class ObjectHuntNode(Node):

    def __init__(self):
        super().__init__('object_hunt')

        self.model  = YOLO(YOLO_MODEL)
        self.bridge = CvBridge()
        self.get_logger().info(f'YOLOv8 loaded ({YOLO_MODEL})  conf_threshold={CONF_THRESHOLD}')

        # Print all class names once at startup so user knows exact names to type
        self.get_logger().info("YOLO classes: " +
            ", ".join(self.model.names[i] for i in sorted(self.model.names)))

        self.state         = State.IDLE
        self.target_object = ""
        self._frame_count  = 0   # for debug throttling

        self.x, self.y       = 0.0, 0.0
        self.yaw, self.yaw_prev = 0.0, 0.0
        self.odom_ready       = False
        self.cumulative_yaw   = 0.0

        self.scan_yaw_baseline      = 0.0
        self.explore_target_yaw     = 0.0
        self.explore_phase          = 'turn'
        self.explore_start_xy       = None
        self.seen                   = set()
        self.lost_counter           = 0
        self.last_pixel_error       = 0.0
        self.reacquire_direction    = 1.0
        self.reacquire_yaw_baseline = 0.0

        self.lock          = threading.Lock()
        self.latest_frame  = None
        self.latest_depth  = None

        self.front_dist      = 10.0
        self.path_clear_dist = 10.0
        self.spin_clear_dist = 10.0
        self.full_ranges     = np.ones(360, dtype=np.float32) * 10.0

        self.create_subscription(Image,     'camera/image',       self._rgb_cb,   1)
        self.create_subscription(Image,     'camera/depth/image', self._depth_cb, 1)
        self.create_subscription(LaserScan, '/scan',              self._scan_cb,  10)
        self.create_subscription(Odometry,  '/odom',              self._odom_cb,  10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.running = True
        threading.Thread(target=self._spin_loop,  daemon=True).start()
        threading.Thread(target=self._input_loop, daemon=True).start()

    # ══════════════════════════════════════════════════════════════
    #  UTILITIES
    # ══════════════════════════════════════════════════════════════
    def _move(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._move(0.0, 0.0)

    @staticmethod
    def _normalize_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ══════════════════════════════════════════════════════════════
    #  SENSOR CALLBACKS
    # ══════════════════════════════════════════════════════════════
    def _rgb_cb(self, msg):
        with self.lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _depth_cb(self, msg):
        with self.lock:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='32FC1')

    def _scan_cb(self, msg):
        r = np.array(msg.ranges, dtype=np.float32)
        r[~np.isfinite(r) | (r <= 0)] = 10.0
        n = len(r)
        if n != 360:
            r = r[np.linspace(0, n-1, 360).astype(int)]
        self.full_ranges     = r
        self.front_dist      = float(np.min(np.concatenate((r[:20], r[-20:]))))
        self.path_clear_dist = self._compute_path_clearance()
        # Use 10th-percentile not min — avoids chassis self-reflection and
        # single noisy pixels triggering the backing-off loop
        self.spin_clear_dist = float(np.percentile(r, 10))

    def _compute_path_clearance(self):
        r, n  = self.full_ranges, len(self.full_ranges)
        band  = ROBOT_HALF_WIDTH + SAFETY_MARGIN
        best  = 10.0
        for deg in range(-90, 91, 2):
            d   = float(r[deg % n])
            fwd = d * math.cos(math.radians(deg))
            lat = d * math.sin(math.radians(deg))
            if fwd > 0 and abs(lat) <= band and fwd < best:
                best = fwd
        return best

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_now = math.atan2(siny, cosy)
        if not self.odom_ready:
            self.yaw = self.yaw_prev = yaw_now
            self.odom_ready = True
            return
        delta = yaw_now - self.yaw_prev
        if delta >  math.pi: delta -= 2 * math.pi
        if delta < -math.pi: delta += 2 * math.pi
        self.yaw, self.yaw_prev = yaw_now, yaw_now
        self.cumulative_yaw += abs(delta)

    def _mark_seen_from_scan(self):
        r = self.full_ranges
        for deg in range(0, 360, SEEN_RAY_STEP_DEG):
            dist    = min(float(r[deg % 360]), SEEN_MARK_MAX)
            bearing = deg if deg <= 180 else deg - 360
            gb      = self.yaw + math.radians(bearing)
            for s in range(1, max(1, int(dist / GRID_CELL)) + 1):
                d = s * GRID_CELL
                if d > dist: break
                self.seen.add((round((self.x + d*math.cos(gb)) / GRID_CELL),
                               round((self.y + d*math.sin(gb)) / GRID_CELL)))
        self.seen.add((round(self.x/GRID_CELL), round(self.y/GRID_CELL)))

    # ══════════════════════════════════════════════════════════════
    #  INPUT LOOP (Bonus 3)
    # ══════════════════════════════════════════════════════════════
    def _input_loop(self):
        while self.running:
            target = input("\nEnter target object: ").strip().lower()
            if not target:
                continue

            # ── Fuzzy name check against real YOLO class names ────
            # Catches typos and Gazebo model names that don't match
            # COCO exactly (e.g. user types "fridge" → "refrigerator")
            exact_match = target in self.model.names.values()
            if not exact_match:
                close = [v for v in self.model.names.values()
                         if target in v or v in target]
                if close:
                    print(f'  "{target}" not exact. Did you mean: {close}?')
                    print(f'  Using closest: "{close[0]}"')
                    target = close[0]
                else:
                    print(f'  WARNING: "{target}" not in YOLO classes.')
                    print(f'  Available: {[v for v in self.model.names.values()]}')
                    print(f'  Continuing anyway — detection may fail.')

            self._start_mission(target)
            while self.running and self.state != State.COMPLETE:
                time.sleep(0.2)
            time.sleep(1.5)

    def _start_mission(self, target: str):
        self.target_object  = target
        self.seen           = set()
        self.lost_counter   = 0
        self.cumulative_yaw = 0.0
        self._enter_scan()
        self.get_logger().info(f"[MISSION] Hunting: {target}")

    # ══════════════════════════════════════════════════════════════
    #  SEARCH STATES
    # ══════════════════════════════════════════════════════════════
    def _enter_scan(self):
        self.state             = State.SCAN
        self.scan_yaw_baseline = self.cumulative_yaw

    def _enter_reacquire(self):
        self.state                    = State.REACQUIRE
        # Turn toward where the target was drifting last seen.
        # last_pixel_error > 0 means target was to the RIGHT → turn right (negative angular)
        # last_pixel_error < 0 means target was to the LEFT  → turn left  (positive angular)
        self.reacquire_direction      = -1.0 if self.last_pixel_error > 0 else 1.0
        self.reacquire_yaw_baseline   = self.cumulative_yaw

    def _reacquire_step(self, frame):
        """
        Keep rotating in the direction the target was last seen drifting.
        We do NOT stop at an angle limit — we keep going until YOLO
        actually sees the target again (handled in _process_frame: as soon
        as target_found becomes True, the function returns early through the
        TRACKING/APPROACH branch and never calls this function).
        Only fall back to a full SCAN if we have done a complete 360°
        without finding it — meaning it genuinely moved or we misjudged
        the direction.
        """
        if self.spin_clear_dist < ROTATE_SAFE_RADIUS:
            self._move(-0.08, 0.0)
            self._draw_dashboard(frame, "Too tight - backing off")
            return

        self._move(0.0, self.reacquire_direction * SCAN_ANGULAR_SPEED)
        rotated = self.cumulative_yaw - self.reacquire_yaw_baseline

        self._draw_dashboard(frame,
            f"Reacquiring — rotated {math.degrees(rotated):.0f}° (rotating until found)")

        # Only give up and do a full scan after a complete 360°
        if rotated >= 2 * math.pi:
            self._enter_scan()

    def _scan_step(self, frame):
        if self.spin_clear_dist < ROTATE_SAFE_RADIUS:
            self._move(-0.08, 0.0)
            self._draw_dashboard(frame, "Too tight - backing off")
            return
        self._move(0.0, SCAN_ANGULAR_SPEED)
        rotated = self.cumulative_yaw - self.scan_yaw_baseline
        self._draw_dashboard(frame, f"Scanning {math.degrees(rotated):.0f}/360 deg")
        if rotated >= FULL_ROTATION:
            self._stop()
            self._mark_seen_from_scan()
            self.state = State.EXPLORE
            self._begin_explore_leg()

    def _begin_explore_leg(self):
        bearing                = self._choose_explore_direction()
        self.explore_target_yaw = self._normalize_angle(self.yaw + bearing)
        self.explore_phase     = 'turn'
        self.explore_start_xy  = None

    def _choose_explore_direction(self):
        r, n = self.full_ranges, len(self.full_ranges)
        best_score, best_deg        = -1e9, 0
        fallback_score, fallback_deg = -1e9, 0
        for deg in range(-175, 180, CANDIDATE_STEP_DEG):
            idxs     = [(deg % 360 + off) % n for off in range(-WINDOW_DEG, WINDOW_DEG+1)]
            openness = float(np.mean(r[idxs]))
            gb       = self.yaw + math.radians(deg)
            fd       = min(openness, EXPLORE_HORIZON)
            fc       = (round((self.x + fd*math.cos(gb)) / GRID_CELL),
                        round((self.y + fd*math.sin(gb)) / GRID_CELL))
            is_frontier = fc not in self.seen
            score = (openness + UNSEEN_BONUS) if is_frontier else min(openness, OPENNESS_CAP)
            if score > fallback_score:
                fallback_score, fallback_deg = score, deg
            if openness >= MIN_OPEN_DIST and score > best_score:
                best_score, best_deg = score, deg
        return math.radians(best_deg if best_score > -1e9 else fallback_deg)

    def _explore_step(self, frame):
        if self.explore_phase == 'turn':
            if self.spin_clear_dist < ROTATE_SAFE_RADIUS:
                self._move(-0.08, 0.0)
                self._draw_dashboard(frame, "Too tight - backing off")
                return
            err = self._normalize_angle(self.explore_target_yaw - self.yaw)
            if abs(err) < EXPLORE_TURN_TOL:
                self.explore_phase    = 'drive'
                self.explore_start_xy = (self.x, self.y)
            else:
                self._move(0.0, max(-1.0, min(1.0, EXPLORE_TURN_KP * err)))
                self._draw_dashboard(frame, "Turning toward open space")
            return

        if self.path_clear_dist < OBSTACLE_FRONT:
            self._stop()
            self._enter_scan()
            return
        traveled = math.hypot(self.x - self.explore_start_xy[0],
                              self.y - self.explore_start_xy[1])
        if traveled >= MAX_EXPLORE_DIST:
            self._stop(); self._enter_scan(); return

        if self.path_clear_dist < EXPLORE_SLOW_DIST:
            scale = (self.path_clear_dist - OBSTACLE_FRONT) / (EXPLORE_SLOW_DIST - OBSTACLE_FRONT)
            speed = max(0.08, EXPLORE_LINEAR * scale)
        else:
            speed = EXPLORE_LINEAR

        err  = self._normalize_angle(self.explore_target_yaw - self.yaw)
        turn = max(-0.6, min(0.6, EXPLORE_DRIVE_KP * err))
        self._move(speed, turn)
        self._draw_dashboard(frame, f"Exploring {traveled:.1f} m")

    # ══════════════════════════════════════════════════════════════
    #  BONUS 1 — ROBUST DEPTH
    # ══════════════════════════════════════════════════════════════
    def _get_depth(self, cx, cy, depth_img):
        h, w  = depth_img.shape
        half  = DEPTH_PATCH // 2
        r0, r1 = max(cy-half,0), min(cy+half+1,h)
        c0, c1 = max(cx-half,0), min(cx+half+1,w)
        patch  = depth_img[r0:r1, c0:c1].flatten()
        valid  = patch[np.isfinite(patch) & (patch > 0.05)]
        return float(np.median(valid)) if len(valid) else None

    # ══════════════════════════════════════════════════════════════
    #  CORE FRAME PROCESSING
    # ══════════════════════════════════════════════════════════════
    def _process_frame(self, frame, depth):
        self._frame_count += 1

        if self.state == State.IDLE:
            self._draw_dashboard(frame, "Waiting for target")
            return frame

        if self.state == State.COMPLETE:
            self._draw_dashboard(frame, "Mission Complete!")
            return frame

        # ── YOLO ───────────────────────────────────────────────────
        results      = self.model(frame, conf=CONF_THRESHOLD, verbose=False)
        target_found = False
        best         = None
        all_detections = []   # for debug

        for result in results:
            for box in result.boxes:
                cid  = int(box.cls[0])
                name = self.model.names[cid].lower()
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                all_detections.append((name, conf))

                # Draw all detections dimly in grey
                cv2.rectangle(frame, (x1,y1), (x2,y2), (60,60,60), 1)
                cv2.putText(frame, f"{name} {conf:.2f}", (x1, y1-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100,100,100), 1)

                if name != self.target_object:
                    continue

                target_found = True
                area = (x2-x1)*(y2-y1)
                if best is None or area > best[0]:
                    cx, cy = (x1+x2)//2, (y1+y2)//2
                    d = self._get_depth(cx, cy, depth) if depth is not None else None
                    best = (area, x1,y1,x2,y2, cx,cy, conf, d)

        # ── DEBUG: print what YOLO sees every N frames ─────────────
        if DEBUG_DETECTIONS and self._frame_count % DEBUG_PRINT_EVERY == 0:
            if all_detections:
                det_str = ", ".join(f"{n}({c:.2f})" for n,c in all_detections)
                self.get_logger().info(f"[YOLO] Detected: {det_str}")
            else:
                self.get_logger().info(f"[YOLO] Nothing detected (conf>{CONF_THRESHOLD})")

        # ── TARGET NOT VISIBLE ─────────────────────────────────────
        if not target_found:
            if self.state == State.APPROACH and self.front_dist < STOP_DISTANCE:
                self._stop()
                self.state = State.COMPLETE
                self.get_logger().info("Mission Complete (lidar-confirmed)")
                self._draw_dashboard(frame, f"TARGET REACHED (~{self.front_dist:.2f}m, lidar)")
                return frame

            if self.state in (State.TRACKING, State.APPROACH):
                self.lost_counter += 1
                if self.lost_counter < LOST_GRACE_FRAMES:
                    # Creep forward slightly + keep turning the same direction
                    # we were correcting toward. This changes the viewing angle
                    # so YOLO gets a fresh look rather than staring at the same
                    # partial/side view that caused the dropout.
                    nudge_angular = -KP_ANGULAR * self.last_pixel_error * 0.5
                    nudge_linear  = 0.08 if self.path_clear_dist > 0.8 else 0.0
                    self._move(nudge_linear, nudge_angular)
                    self._draw_dashboard(frame,
                        f"Flickered — nudging {self.lost_counter}/{LOST_GRACE_FRAMES}")
                    return frame
                self._enter_reacquire()

            if   self.state == State.SCAN:      self._scan_step(frame)
            elif self.state == State.REACQUIRE:  self._reacquire_step(frame)
            elif self.state == State.EXPLORE:    self._explore_step(frame)
            else:                                self._enter_scan()
            return frame

        # ── TARGET VISIBLE ─────────────────────────────────────────
        self.lost_counter = 0
        _, x1,y1,x2,y2, cx,cy, conf, distance = best

        img_cx      = frame.shape[1] // 2
        pixel_error = cx - img_cx
        self.last_pixel_error = pixel_error

        # Draw target box prominently
        cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 3)
        cv2.circle(frame, (cx,cy), 7, (0,255,0), -1)
        cv2.line(frame, (img_cx, frame.shape[0]//2), (cx,cy), (0,200,255), 1)
        lbl = f"{self.target_object} {conf:.2f}"
        if distance: lbl += f"  {distance:.2f}m"
        (lw,lh), base = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        cv2.rectangle(frame, (x1,y1-lh-base-6), (x1+lw,y1), (0,180,0), -1)
        cv2.putText(frame, lbl, (x1,y1-base-2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,0,0), 2)

        # Stage 6 — mission complete
        eff_dist = distance if distance is not None else self.front_dist
        if (
            eff_dist is not None
            and abs(eff_dist - FOLLOW_DISTANCE) < FOLLOW_TOLERANCE
        ):
            self._stop()
            self.state = State.COMPLETE
            self.get_logger().info(
                f"Mission Complete — Holding target at {eff_dist:.2f} m"
            )
            self._draw_dashboard(frame, f"Holding at {eff_dist:.2f} m")
            return frame

        # Stage 5 — align then approach
        if abs(pixel_error) > CENTER_TOL_PX:
            self.state = State.TRACKING
            self._move(0.0, -KP_ANGULAR * pixel_error)
            self._draw_dashboard(frame, f"Aligning err={pixel_error:+d}px", distance)
        else:
            self.state = State.APPROACH

            if self.path_clear_dist < 0.4:
               self._move(-0.1, 0.0)

            elif distance is not None:
                 error = distance - FOLLOW_DISTANCE

        # Already at desired distance
                 if abs(error) < FOLLOW_TOLERANCE:
                    self._move(0.0, -KP_ANGULAR * 0.4 * pixel_error)

        # Too far -> move forward
                 elif error > 0:
                      speed = min(SPEED_FAST, 0.5 * error)
                      speed = max(speed, SPEED_SLOW)
                      self._move(speed, -KP_ANGULAR * 0.4 * pixel_error)

        # Too close -> move backward
                 else:
                      self._move(-0.08, -KP_ANGULAR * 0.4 * pixel_error)

            else:
        # Depth unavailable
                 self._move(SPEED_SLOW, -KP_ANGULAR * 0.4 * pixel_error)

        self._draw_dashboard(frame, "Maintaining Distance", distance)

        return frame

    # ══════════════════════════════════════════════════════════════
    #  BONUS 2 — DASHBOARD
    # ══════════════════════════════════════════════════════════════
    def _draw_dashboard(self, frame, mode="", distance=None):
        STATE_COL = {
            State.IDLE:      (160,160,160), State.SCAN:      (0,200,255),
            State.EXPLORE:   (0,165,255),   State.REACQUIRE: (0,120,255),
            State.TRACKING:  (0,200,255),   State.APPROACH:  (0,255,0),
            State.COMPLETE:  (50,255,80),
        }
        col = STATE_COL.get(self.state, (200,200,200))
        overlay = frame.copy()
        cv2.rectangle(overlay, (5,5), (345,220), (10,10,10), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        def put(txt, row, c=(210,210,210), b=1):
            cv2.putText(frame, txt, (12, 28+row*26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.54, c, b)

        put(f"TARGET   : {self.target_object.upper() or '---'}", 0)
        put(f"STATE    : {self.state}", 1, col, 2)
        put(f"MODE     : {mode[:40]}", 2)
        put(f"DISTANCE : {f'{distance:.2f} m' if distance else '---'}", 3)
        put(f"CONF THR : {CONF_THRESHOLD}  (lower = easier detect)", 4)
        put(f"EXPLORED : {len(self.seen)} cells", 5)
        put(f"FRONT={self.front_dist:.2f}m  CLEAR={self.path_clear_dist:.2f}m", 6)
        put(f"SPIN ={self.spin_clear_dist:.2f}m (need>={ROTATE_SAFE_RADIUS:.2f})", 7)

    # ══════════════════════════════════════════════════════════════
    #  DISPLAY + SPIN LOOPS
    # ══════════════════════════════════════════════════════════════
    def display_image(self):
        cv2.namedWindow('Object Hunt', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Object Hunt', 920, 680)
        while rclpy.ok() and self.running:
            with self.lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None
                depth = self.latest_depth.copy() if self.latest_depth is not None else None
            if frame is not None:
                cv2.imshow('Object Hunt', self._process_frame(frame, depth))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break
        cv2.destroyAllWindows()
        self.running = False

    def _spin_loop(self):
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        self.running = False
        self._stop()


def main(args=None):
    print(f'OpenCV {cv2.__version__}')
    rclpy.init(args=args)
    node = ObjectHuntNode()
    try:
        node.display_image()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
