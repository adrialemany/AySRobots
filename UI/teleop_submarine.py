#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AysRoboTS • Submarine Teleop Console (ROS2 Humble + PySide6)

✅ Features
- Subscribes to Unity camera (/camera/image/compressed or /camera/image_raw)
- Subscribes to LiDAR 4D (/lidar/points) and computes a "RADAR" (F/L/R/B/D)
- Runs YOLO locally on the received frames (Ultralytics)
- Select a YOLO target and FOLLOW: keeps it centered + approaches to user distance (LiDAR front safety)
- Incremental keyboard teleop (hold key ramps via auto-repeat), Space = all zero
- Clean shutdown on Ctrl+C + window close
- "RECOVER" button (press-and-hold): publishes /relocate = True continuously while pressed, False on release

Expected topics:
  /camera/image/compressed   sensor_msgs/msg/CompressedImage
  /camera/image_raw          sensor_msgs/msg/Image
  /lidar/points              sensor_msgs/msg/PointCloud2
  /cmd_vel                   geometry_msgs/msg/Twist
  /relocate                  std_msgs/msg/Bool   (press & hold true)

Install:
  sudo apt install ros-humble-cv-bridge
  python3 -m pip install --user -U PySide6 ultralytics opencv-python numpy sensor_msgs_py
"""

import sys
import time
import math
import threading
import signal
from dataclasses import dataclass
from typing import Optional, Any, Dict, List, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge

from std_msgs.msg import String, Bool

from PySide6 import QtCore, QtGui, QtWidgets

# --- YOLO (local inference) ---
try:
    from ultralytics import YOLO
    ULTRALYTICS_OK = True
except Exception:
    ULTRALYTICS_OK = False


# -------------------------
# Tunables
# -------------------------
YOLO_ENABLED = True
YOLO_MODEL = "yolov8n.pt"
YOLO_CONF = 0.35
YOLO_IMGSZ = 640
YOLO_DEVICE = None  # None, "cpu", "cuda:0", ...

# If yaw feels inverted, flip between +1.0 and -1.0
YAW_SIGN = -1.0


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def now_s() -> float:
    return time.time()


# -------------------------
# ROS Node
# -------------------------
class TeleopRosNode(Node):
    def __init__(self):
        super().__init__("teleop_submarine_ui")
        self.bridge = CvBridge()

        # Topics
        self.cmd_vel = "/cmd_vel"
        self.mode_out = "/teleop/mode"          # optional
        self.relocate_topic = "/relocate"       # press&hold button publishes Bool

        self.camera_raw_in = "/camera/image_raw"
        self.camera_comp_in = "/camera/image/compressed"
        self.lidar_in = "/lidar/points"

        # Publishers
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel, 10)
        self.pub_mode = self.create_publisher(String, self.mode_out, 10)
        self.pub_relocate = self.create_publisher(Bool, self.relocate_topic, 10)

        # Subscriptions
        self.create_subscription(Image, self.camera_raw_in, self._on_image_raw, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, self.camera_comp_in, self._on_image_compressed, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.lidar_in, self._on_lidar, qos_profile_sensor_data)

        # Monitor bus /cmd_vel (debug)
        self.create_subscription(Twist, self.cmd_vel, self._on_cmd_in, 10)

        self._lock = threading.Lock()

        self._last_bgr: Optional[np.ndarray] = None
        self._last_img_stamp: float = 0.0
        self._last_img_src: str = "---"
        self._frame_seq: int = 0

        # nearest frontal (legacy sonar)
        self._nearest_xyz: Optional[Tuple[float, float, float]] = None
        self._nearest_dist: Optional[float] = None
        self._nearest_stamp: float = 0.0

        # RADAR distances (axis-projected)
        self._radar: Dict[str, Optional[float]] = {"front": None, "left": None, "right": None, "back": None, "down": None}
        self._radar_stamp: float = 0.0

        # /cmd_vel bus
        self._cmd_in_last: Optional[Tuple[float, float, float, float]] = None
        self._cmd_in_stamp: float = 0.0

    def set_mode(self, mode: str):
        s = String()
        s.data = mode.upper()
        self.pub_mode.publish(s)

    def publish_cmd_vel(self, vx: float, vy: float, vz: float, yaw: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw * YAW_SIGN)  # apply sign ONCE here
        self.pub_cmd.publish(msg)

    def publish_relocate(self, is_true: bool):
        msg = Bool()
        msg.data = bool(is_true)
        self.pub_relocate.publish(msg)

    def get_latest(self):
        with self._lock:
            bgr = None if self._last_bgr is None else self._last_bgr.copy()
            img_age = now_s() - self._last_img_stamp if self._last_img_stamp > 0 else None
            img_src = self._last_img_src
            frame_seq = self._frame_seq

            nearest_dist = self._nearest_dist
            nearest_xyz = self._nearest_xyz
            nearest_age = now_s() - self._nearest_stamp if self._nearest_stamp > 0 else None

            radar = self._radar.copy()
            radar_age = now_s() - self._radar_stamp if self._radar_stamp > 0 else None

            cmd_in = self._cmd_in_last
            cmd_in_age = now_s() - self._cmd_in_stamp if self._cmd_in_stamp > 0 else None

        return (bgr, img_age, img_src, frame_seq,
                nearest_dist, nearest_xyz, nearest_age,
                cmd_in, cmd_in_age,
                radar, radar_age)

    # Camera
    def _store_bgr(self, bgr: np.ndarray, src: str):
        with self._lock:
            self._last_bgr = bgr
            self._last_img_stamp = now_s()
            self._last_img_src = src
            self._frame_seq += 1

    def _on_image_raw(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._store_bgr(bgr, "raw")
        except Exception:
            pass

    def _on_image_compressed(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if bgr is not None:
                self._store_bgr(bgr, "compressed")
        except Exception:
            pass

    # LiDAR -> nearest frontal + RADAR
    def _on_lidar(self, msg: PointCloud2):
        # nearest frontal (euclidean)
        min_d2 = None
        min_xyz = None

        # RADAR (axis-projected) minima
        rad = {"front": None, "left": None, "right": None, "back": None, "down": None}

        max_points = 12000
        min_range = 0.35
        min_r2 = min_range * min_range

        # thresholds (assuming x forward, y left, z up)
        x_front = 0.15
        x_back = -0.15
        y_side = 0.15
        z_down = -0.10

        # clamps to ignore crazy points (optional)
        z_min, z_max = -5.0, 5.0

        try:
            count = 0
            for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                if z < z_min or z > z_max:
                    continue

                d2 = x*x + y*y + z*z
                if d2 < min_r2:
                    continue

                # --- RADAR per direction (distance projected on axis) ---
                # FRONT: min x ahead
                if x > x_front and abs(y) < 2.0:
                    if rad["front"] is None or x < rad["front"]:
                        rad["front"] = float(x)

                # BACK: min |x| behind
                if x < x_back and abs(y) < 2.0:
                    bx = float(abs(x))
                    if rad["back"] is None or bx < rad["back"]:
                        rad["back"] = bx

                # LEFT: min y on left (y>0)
                if y > y_side and abs(x) < 3.0:
                    if rad["left"] is None or y < rad["left"]:
                        rad["left"] = float(y)

                # RIGHT: min |y| on right (y<0)
                if y < -y_side and abs(x) < 3.0:
                    ry = float(abs(y))
                    if rad["right"] is None or ry < rad["right"]:
                        rad["right"] = ry

                # DOWN: min |z| below (z<0)  -> varies when going down IF lidar sees below
                if z < z_down and abs(x) < 3.0 and abs(y) < 3.0:
                    dz = float(abs(z))
                    if rad["down"] is None or dz < rad["down"]:
                        rad["down"] = dz

                # --- nearest frontal sonar (stricter region) ---
                if x > 0.20 and (-0.35 <= z <= 0.35):
                    if (min_d2 is None) or (d2 < min_d2):
                        min_d2 = d2
                        min_xyz = (float(x), float(y), float(z))

                count += 1
                if count >= max_points:
                    break

            with self._lock:
                if min_d2 is not None:
                    self._nearest_xyz = min_xyz
                    self._nearest_dist = float(math.sqrt(min_d2))
                    self._nearest_stamp = now_s()

                self._radar = rad
                self._radar_stamp = now_s()

        except Exception:
            return

    def _on_cmd_in(self, msg: Twist):
        with self._lock:
            self._cmd_in_last = (float(msg.linear.x), float(msg.linear.y), float(msg.linear.z), float(msg.angular.z))
            self._cmd_in_stamp = now_s()


# -------------------------
# YOLO Worker (Qt thread)
# -------------------------
class YoloWorker(QtCore.QObject):
    detections_ready = QtCore.Signal(list, float)  # dets, infer_ms
    status_ready = QtCore.Signal(str)

    def __init__(self, model_name=YOLO_MODEL, conf=YOLO_CONF, imgsz=YOLO_IMGSZ, device=YOLO_DEVICE):
        super().__init__()
        self.model_name = model_name
        self.conf = conf
        self.imgsz = imgsz
        self.device = device

        self._stop = False
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None

        self._model = None
        self._model_ok = False

    def stop(self):
        self._stop = True

    def push_frame(self, bgr: np.ndarray):
        with self._frame_lock:
            self._latest_frame = bgr

    def _load(self):
        if not (ULTRALYTICS_OK and YOLO_ENABLED):
            self._model_ok = False
            self.status_ready.emit("YOLO: disabled / missing ultralytics")
            return
        try:
            self.status_ready.emit(f"YOLO: loading {self.model_name} ...")
            self._model = YOLO(self.model_name)
            if self.device is not None:
                self._model.to(self.device)
            self._model_ok = True
            self.status_ready.emit(f"YOLO: ready ({self.model_name})")
        except Exception as e:
            self._model_ok = False
            self.status_ready.emit(f"YOLO: load failed ({e})")

    def run(self):
        self._load()

        while not self._stop:
            frame = None
            with self._frame_lock:
                if self._latest_frame is not None:
                    frame = self._latest_frame
                    self._latest_frame = None

            if frame is None:
                time.sleep(0.01)
                continue

            if not self._model_ok:
                time.sleep(0.2)
                continue

            t0 = time.time()
            dets_out: List[Dict[str, Any]] = []

            try:
                results = self._model.predict(
                    source=frame,
                    conf=self.conf,
                    imgsz=self.imgsz,
                    verbose=False
                )
                r = results[0]
                names = r.names if hasattr(r, "names") else {}

                if r.boxes is not None and len(r.boxes) > 0:
                    boxes = r.boxes
                    xyxy = boxes.xyxy.cpu().numpy()
                    confs = boxes.conf.cpu().numpy()
                    clss = boxes.cls.cpu().numpy().astype(int)

                    for (x1, y1, x2, y2), c, cls_id in zip(xyxy, confs, clss):
                        label = names.get(cls_id, str(cls_id))
                        cx = float((x1 + x2) * 0.5)
                        cy = float((y1 + y2) * 0.5)
                        dets_out.append({
                            "label": str(label),
                            "conf": float(c),
                            "bbox": (float(x1), float(y1), float(x2), float(y2)),
                            "cx": cx,
                            "cy": cy
                        })

                dets_out.sort(key=lambda d: d["conf"], reverse=True)

            except Exception:
                dets_out = []

            infer_ms = (time.time() - t0) * 1000.0
            self.detections_ready.emit(dets_out, infer_ms)
            time.sleep(0.01)


# -------------------------
# Video widget (fullscreen stable)
# -------------------------
class VideoWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(640, 360)
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent, True)
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, True)

        self._qimg: Optional[QtGui.QImage] = None
        self._scaled_cache: Optional[QtGui.QImage] = None
        self._cache_size = QtCore.QSize(-1, -1)
        self._status_text = "Waiting for camera..."

    def set_status(self, text: str):
        self._status_text = text
        self.update()

    def set_frame(self, bgr: np.ndarray):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        self._qimg = QtGui.QImage(rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888).copy()
        self._scaled_cache = None
        self.update()

    def resizeEvent(self, event: QtGui.QResizeEvent):
        super().resizeEvent(event)
        self._scaled_cache = None

    def paintEvent(self, event: QtGui.QPaintEvent):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        p.fillRect(self.rect(), QtGui.QColor("#0b0f14"))

        if self._qimg is None:
            p.setPen(QtGui.QColor("#9aa4b2"))
            p.drawText(self.rect(), QtCore.Qt.AlignCenter, self._status_text)
            p.end()
            return

        target = self.size()
        if self._scaled_cache is None or self._cache_size != target:
            self._cache_size = QtCore.QSize(target.width(), target.height())
            self._scaled_cache = self._qimg.scaled(
                target, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
            )

        img_w = self._scaled_cache.width()
        img_h = self._scaled_cache.height()
        x = (self.width() - img_w) // 2
        y = (self.height() - img_h) // 2
        p.drawImage(QtCore.QPoint(x, y), self._scaled_cache)

        p.setPen(QtGui.QColor("#1f2a37"))
        p.drawRect(self.rect().adjusted(0, 0, -1, -1))
        p.end()


# -------------------------
# Config
# -------------------------
@dataclass
class TeleopConfig:
    max_lin: float = 2.0
    max_lat: float = 1.2
    max_vz: float = 1.2
    max_yaw: float = 8.0

    step_lin: float = 0.10
    step_lat: float = 0.08
    step_vz: float = 0.08
    step_yaw: float = 0.80  # aggressive turn

    turbo_mult: float = 1.7
    cmd_rate_hz: float = 300.0

    deadman_timeout_s: float = 0.35
    brake_per_s: float = 3.0

    warn_dist_m: float = 1.2
    danger_dist_m: float = 0.6

    draw_yolo: bool = True


# -------------------------
# Main Window
# -------------------------
class TeleopWindow(QtWidgets.QMainWindow):
    def __init__(self, ros_node: TeleopRosNode, cfg: TeleopConfig):
        super().__init__()
        self.ros = ros_node
        self.cfg = cfg

        self.setWindowTitle("AysRoboTS • Submarine Teleop Console")
        self.resize(1420, 820)

        self.keys_down = set()
        self.last_input_time = 0.0
        self.mode = "HOLD"

        self.vx = self.vy = self.vz = self.yaw = 0.0
        self._last_tick = now_s()

        self._last_frame_seq_seen = -1

        self.yolo_dets: List[Dict[str, Any]] = []
        self.yolo_infer_ms: Optional[float] = None
        self.yolo_status: str = "YOLO: ---"

        self.selected_idx = -1
        self.follow_enabled = False

        # RECOVER press&hold publisher timer (True while pressed)
        self._recover_timer = QtCore.QTimer(self)
        self._recover_timer.setInterval(50)  # 20 Hz
        self._recover_timer.timeout.connect(self._recover_tick)
        self._recover_is_pressed = False

        # Build UI
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QHBoxLayout(root)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        layout.addWidget(self._build_left(), 0)
        layout.addWidget(self._build_center(), 1)
        layout.addWidget(self._build_right(), 0)

        self._apply_styles()
        self.set_mode("HOLD")

        # Timers
        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self.refresh_ui)
        self.ui_timer.start(50)

        self.cmd_timer = QtCore.QTimer(self)
        self.cmd_timer.timeout.connect(self.publish_cmd)
        self.cmd_timer.start(int(1000.0 / self.cfg.cmd_rate_hz))

        # YOLO thread
        self.yolo_thread = QtCore.QThread(self)
        self.yolo_worker = YoloWorker()
        self.yolo_worker.moveToThread(self.yolo_thread)
        self.yolo_thread.started.connect(self.yolo_worker.run)
        self.yolo_worker.detections_ready.connect(self._on_yolo_ready)
        self.yolo_worker.status_ready.connect(self._on_yolo_status)
        self.yolo_thread.start()

    def closeEvent(self, event):
        # Ensure everything stops cleanly (including /relocate false)
        try:
            self._stop_recover()
            self.ros.publish_relocate(False)
        except Exception:
            pass

        try:
            self.set_mode("HOLD")
            self.ros.publish_cmd_vel(0, 0, 0, 0)
        except Exception:
            pass

        try:
            self.yolo_worker.stop()
            self.yolo_thread.quit()
            self.yolo_thread.wait(1500)
        except Exception:
            pass

        event.accept()

    # ---- UI build ----
    def _build_left(self):
        w = QtWidgets.QFrame(); w.setObjectName("panel"); w.setMinimumWidth(310)
        v = QtWidgets.QVBoxLayout(w); v.setSpacing(10)

        title = QtWidgets.QLabel("CONTROL"); title.setObjectName("panelTitle")

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_manual = QtWidgets.QPushButton("MANUAL")
        self.btn_auto = QtWidgets.QPushButton("AUTO")
        self.btn_hold = QtWidgets.QPushButton("HOLD")
        for b in (self.btn_manual, self.btn_auto, self.btn_hold):
            b.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
            b.setMinimumHeight(38)
        self.btn_manual.clicked.connect(lambda: self.set_mode("MANUAL"))
        self.btn_auto.clicked.connect(lambda: self.set_mode("AUTO"))
        self.btn_hold.clicked.connect(lambda: self.set_mode("HOLD"))
        btn_row.addWidget(self.btn_manual); btn_row.addWidget(self.btn_auto); btn_row.addWidget(self.btn_hold)

        self.cb_overlay = QtWidgets.QCheckBox("Draw YOLO overlay on video")
        self.cb_overlay.setChecked(self.cfg.draw_yolo)
        self.cb_overlay.stateChanged.connect(lambda _: self._toggle_overlay())

        hint = QtWidgets.QLabel(
            "Incremental teleop:\n"
            "Hold key = ramps up via repeats\n\n"
            "W/S: forward/back\n"
            "A/D: yaw left/right\n"
            "Q/E: strafe left/right\n"
            "R/F: up/down\n"
            "Shift: TURBO\n"
            "Space: ALL ZERO\n"
        )
        hint.setObjectName("hint")

        self.btn_estop = QtWidgets.QPushButton("E-STOP")
        self.btn_estop.setObjectName("estop")
        self.btn_estop.setMinimumHeight(46)
        self.btn_estop.clicked.connect(self.estop)

        v.addWidget(title)
        v.addLayout(btn_row)
        v.addWidget(self.cb_overlay)
        v.addWidget(hint)
        v.addStretch(1)
        v.addWidget(self.btn_estop)
        return w

    def _build_center(self):
        w = QtWidgets.QFrame(); w.setObjectName("panel")
        v = QtWidgets.QVBoxLayout(w); v.setSpacing(10)

        header = QtWidgets.QHBoxLayout()
        self.lbl_status = QtWidgets.QLabel("STATUS: ---"); self.lbl_status.setObjectName("status")
        self.lbl_cam = QtWidgets.QLabel("CAM: ---"); self.lbl_cam.setObjectName("chip")
        self.lbl_ros = QtWidgets.QLabel("ROS: OK"); self.lbl_ros.setObjectName("chip")
        header.addWidget(self.lbl_status, 1); header.addWidget(self.lbl_cam, 0); header.addWidget(self.lbl_ros, 0)

        self.video = VideoWidget()
        self.video.set_status("Waiting for /camera/image/...")

        self.lbl_bottom = QtWidgets.QLabel("Telemetry: ---"); self.lbl_bottom.setObjectName("bottom")

        v.addLayout(header)
        v.addWidget(self.video, 1)
        v.addWidget(self.lbl_bottom)
        return w

    def _build_right(self):
        w = QtWidgets.QFrame(); w.setObjectName("panel"); w.setMinimumWidth(440)
        v = QtWidgets.QVBoxLayout(w); v.setSpacing(10)

        title = QtWidgets.QLabel("PERCEPTION"); title.setObjectName("panelTitle")

        self.lbl_sonar = QtWidgets.QLabel("SONAR: ---"); self.lbl_sonar.setObjectName("sonarCard")
        self.lbl_radar = QtWidgets.QLabel("RADAR: ---"); self.lbl_radar.setObjectName("radarCard")

        # RECOVER hold button
        self.btn_recover = QtWidgets.QPushButton("RECOVER (hold)")
        self.btn_recover.setObjectName("recover")
        self.btn_recover.setMinimumHeight(40)
        self.btn_recover.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.btn_recover.pressed.connect(self._start_recover)
        self.btn_recover.released.connect(self._stop_recover)

        self.lbl_targets = QtWidgets.QLabel("TARGETS: ---"); self.lbl_targets.setObjectName("targetsCard")
        self.lbl_yolo_status = QtWidgets.QLabel("YOLO: ---"); self.lbl_yolo_status.setObjectName("hint")

        legend = QtWidgets.QLabel("Detections (click to select)"); legend.setObjectName("hint")
        self.yolo_list = QtWidgets.QListWidget(); self.yolo_list.setObjectName("list")
        self.yolo_list.currentRowChanged.connect(self._on_select_target)

        self.lbl_sel = QtWidgets.QLabel("SELECTED: ---"); self.lbl_sel.setObjectName("selectedCard")

        row = QtWidgets.QHBoxLayout()
        self.spin_dist = QtWidgets.QDoubleSpinBox()
        self.spin_dist.setRange(0.3, 5.0)
        self.spin_dist.setSingleStep(0.1)
        self.spin_dist.setValue(1.5)
        self.spin_dist.setSuffix(" m")
        self.spin_dist.setToolTip("Desired distance (using LiDAR front / safety).")

        self.btn_follow = QtWidgets.QPushButton("FOLLOW")
        self.btn_stop_follow = QtWidgets.QPushButton("STOP")
        for b in (self.btn_follow, self.btn_stop_follow):
            b.setMinimumHeight(36)
            b.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.btn_follow.clicked.connect(self._start_follow)
        self.btn_stop_follow.clicked.connect(self._stop_follow)

        row.addWidget(self.spin_dist)
        row.addWidget(self.btn_follow)
        row.addWidget(self.btn_stop_follow)

        v.addWidget(title)
        v.addWidget(self.lbl_sonar)
        v.addWidget(self.lbl_radar)
        v.addWidget(self.btn_recover)
        v.addWidget(self.lbl_targets)
        v.addWidget(self.lbl_yolo_status)
        v.addWidget(legend)
        v.addWidget(self.yolo_list, 1)
        v.addWidget(self.lbl_sel)
        v.addLayout(row)
        return w

    def _apply_styles(self):
        self.setStyleSheet("""
        QMainWindow { background: #070a0f; }
        #panel { background: #0b0f14; border: 1px solid #1f2a37; border-radius: 14px; }
        #panelTitle { color: #e5e7eb; font-size: 16px; font-weight: 900; letter-spacing: 1px; padding: 8px 8px 0 8px; }

        QPushButton { background: #111827; border: 1px solid #263445; border-radius: 10px; color: #e5e7eb; font-weight: 800; }
        QPushButton:hover { border-color: #3b82f6; }
        QPushButton:pressed { background: #0b1220; }

        QPushButton#estop { background: #1a0b0b; border: 1px solid #7f1d1d; color: #fecaca; font-weight: 900; }
        QPushButton#estop:hover { border-color: #ef4444; }

        QPushButton#recover { background: #0b1a12; border: 1px solid #1f6f3a; color: #bbf7d0; font-weight: 900; }
        QPushButton#recover:hover { border-color: #22c55e; }

        QCheckBox { color: #cbd5e1; padding: 0 10px; }

        #status { color: #e5e7eb; font-weight: 900; font-size: 14px; padding: 6px 10px; }
        #chip { color: #cbd5e1; background: #0f172a; border: 1px solid #22314a; border-radius: 999px; padding: 6px 10px; }
        #bottom { color: #9aa4b2; padding: 6px 10px; }

        #hint { color: #9aa4b2; padding: 0 10px; font-size: 12px; line-height: 16px; }

        #sonarCard { color: #e5e7eb; padding: 10px 12px; background: #08131f; border: 1px solid #1d3552; border-radius: 12px; font-weight: 900; }
        #radarCard { color: #e5e7eb; padding: 10px 12px; background: #071018; border: 1px solid #22314a; border-radius: 12px; font-weight: 900; }
        #targetsCard { color: #e5e7eb; padding: 10px 12px; background: #0f172a; border: 1px solid #22314a; border-radius: 12px; font-weight: 900; }
        #selectedCard { color: #e5e7eb; padding: 10px 12px; background: #0f172a; border: 1px solid #22314a; border-radius: 12px; font-weight: 900; }

        #list { background: #070a0f; border: 1px solid #1f2a37; border-radius: 12px; color: #e5e7eb; padding: 6px; }

        QLabel { font-family: Inter, Segoe UI, Ubuntu, Arial; }
        """)

    # ---- Recover (press&hold) ----
    def _recover_tick(self):
        # publish True continuously while pressed
        try:
            self.ros.publish_relocate(True)
        except Exception:
            pass

    def _start_recover(self):
        self._recover_is_pressed = True
        try:
            self.ros.publish_relocate(True)
        except Exception:
            pass
        self._recover_timer.start()

    def _stop_recover(self):
        self._recover_is_pressed = False
        if self._recover_timer.isActive():
            self._recover_timer.stop()
        try:
            self.ros.publish_relocate(False)
        except Exception:
            pass

    # ---- Helpers ----
    def _toggle_overlay(self):
        self.cfg.draw_yolo = self.cb_overlay.isChecked()

    def set_mode(self, mode: str):
        self.mode = mode.upper()
        try:
            self.ros.set_mode(self.mode)
        except Exception:
            pass
        self._update_mode_buttons()
        self.last_input_time = now_s()
        if self.mode == "HOLD":
            self.follow_enabled = False
            self.all_zero()

    def _update_mode_buttons(self):
        def mark(btn, active: bool):
            btn.setStyleSheet("border-color: #3b82f6;" if active else "")
        mark(self.btn_manual, self.mode == "MANUAL")
        mark(self.btn_auto, self.mode == "AUTO")
        mark(self.btn_hold, self.mode == "HOLD")

    def estop(self):
        self.set_mode("HOLD")

    def all_zero(self):
        self.vx = self.vy = self.vz = self.yaw = 0.0
        try:
            self.ros.publish_cmd_vel(0, 0, 0, 0)
        except Exception:
            pass

    def _status_from_dist(self, d: Optional[float]) -> str:
        if d is None:
            return "NO SONAR"
        if d <= self.cfg.danger_dist_m:
            return "DANGER"
        if d <= self.cfg.warn_dist_m:
            return "CAUTION"
        return "CLEAR"

    @QtCore.Slot(list, float)
    def _on_yolo_ready(self, dets: list, infer_ms: float):
        self.yolo_dets = dets
        self.yolo_infer_ms = infer_ms

    @QtCore.Slot(str)
    def _on_yolo_status(self, txt: str):
        self.yolo_status = txt

    def _on_select_target(self, idx: int):
        self.selected_idx = idx
        self.follow_enabled = False

    def _start_follow(self):
        if 0 <= self.selected_idx < len(self.yolo_dets):
            self.follow_enabled = True
            self.set_mode("AUTO")

    def _stop_follow(self):
        self.follow_enabled = False
        self.set_mode("HOLD")

    def _overlay_yolo(self, bgr: np.ndarray, dets: List[Dict[str, Any]]) -> np.ndarray:
        out = bgr.copy()
        h, w = out.shape[:2]
        for d in dets[:25]:
            bbox = d.get("bbox", None)
            if not bbox:
                continue
            x1, y1, x2, y2 = bbox
            x1 = int(clamp(x1, 0, w - 1)); x2 = int(clamp(x2, 0, w - 1))
            y1 = int(clamp(y1, 0, h - 1)); y2 = int(clamp(y2, 0, h - 1))
            label = d.get("label", "obj")
            conf = float(d.get("conf", 0.0))
            cv2.rectangle(out, (x1, y1), (x2, y2), (245, 245, 245), 2)
            cv2.putText(out, f"{label} {conf:.2f}", (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (245, 245, 245), 1, cv2.LINE_AA)
        return out

    # ---- UI refresh ----
    def refresh_ui(self):
        (bgr, img_age, img_src, frame_seq,
         nearest_dist, nearest_xyz, nearest_age,
         cmd_in, cmd_in_age,
         radar, radar_age) = self.ros.get_latest()

        self.lbl_status.setText(f"STATUS: {self._status_from_dist(nearest_dist)}   |   MODE: {self.mode}")

        if img_age is None:
            self.lbl_cam.setText("CAM: ---")
        else:
            self.lbl_cam.setText(f"CAM({img_src}): {img_age*1000:.0f} ms")

        # SONAR
        if nearest_dist is None or nearest_xyz is None:
            self.lbl_sonar.setText("SONAR: ---")
        else:
            x, y, z = nearest_xyz
            self.lbl_sonar.setText(f"SONAR: {nearest_dist:.2f} m   (x={x:+.2f}, y={y:+.2f}, z={z:+.2f})")

        # RADAR
        def fmt(v):
            return "---" if v is None else f"{v:.2f}m"

        # NOTE: If your lidar is actually a horizontal scan, DOWN may stay '---' (no points below)
        self.lbl_radar.setText(
            "RADAR  "
            f"F:{fmt(radar.get('front'))}  "
            f"L:{fmt(radar.get('left'))}  "
            f"R:{fmt(radar.get('right'))}  "
            f"B:{fmt(radar.get('back'))}  "
            f"D:{fmt(radar.get('down'))}"
        )

        # YOLO status + targets
        self.lbl_yolo_status.setText(self.yolo_status)
        n = len(self.yolo_dets)
        if self.yolo_infer_ms is not None:
            self.lbl_targets.setText(f"TARGETS: {n}   |   YOLO: {self.yolo_infer_ms:.0f} ms")
        else:
            self.lbl_targets.setText(f"TARGETS: {n}")

        # List
        self.yolo_list.blockSignals(True)
        self.yolo_list.clear()
        if YOLO_ENABLED and ULTRALYTICS_OK:
            if self.yolo_dets:
                for d in self.yolo_dets[:30]:
                    self.yolo_list.addItem(f"{d['label']} • {d['conf']:.2f}")
            else:
                self.yolo_list.addItem("No detections (YOLO local)")
        else:
            self.yolo_list.addItem("YOLO disabled / missing ultralytics")
        self.yolo_list.blockSignals(False)

        # Selected
        if self.selected_idx < 0 or self.selected_idx >= len(self.yolo_dets):
            self.lbl_sel.setText("SELECTED: ---")
        else:
            d = self.yolo_dets[self.selected_idx]
            self.lbl_sel.setText(f"SELECTED: {d.get('label','obj')} • {d.get('conf',0.0):.2f}")

        # Push new camera frames to YOLO
        if bgr is not None and frame_seq != self._last_frame_seq_seen:
            self._last_frame_seq_seen = frame_seq
            if YOLO_ENABLED and ULTRALYTICS_OK:
                self.yolo_worker.push_frame(bgr)

        # Video
        if bgr is not None:
            if self.cfg.draw_yolo and self.yolo_dets:
                bgr = self._overlay_yolo(bgr, self.yolo_dets)
            self.video.set_frame(bgr)

        # Telemetry
        parts = [f"OUT: vx={self.vx:+.2f} vy={self.vy:+.2f} vz={self.vz:+.2f} yaw={self.yaw:+.2f}"]
        if cmd_in is not None and cmd_in_age is not None:
            ivx, ivy, ivz, iyaw = cmd_in
            parts.append(f"BUS: vx={ivx:+.2f} vy={ivy:+.2f} vz={ivz:+.2f} yaw={iyaw:+.2f} ({cmd_in_age*1000:.0f}ms)")
        self.lbl_bottom.setText("Telemetry: " + " | ".join(parts))

    # ---- Control publishing ----
    def publish_cmd(self):
        if self.mode == "HOLD":
            return

        t = now_s()
        dt = max(1e-3, t - self._last_tick)
        self._last_tick = t

        if self.mode == "AUTO" and self.follow_enabled:
            self._auto_follow_step()
            return

        # Deadman: if no key activity, brake to zero
        if (t - self.last_input_time) > self.cfg.deadman_timeout_s:
            self._brake_to_zero(dt)
            self.ros.publish_cmd_vel(self.vx, self.vy, self.vz, self.yaw)
            return

        self._brake_missing_axes(dt)
        self.ros.publish_cmd_vel(self.vx, self.vy, self.vz, self.yaw)

    def _brake_to_zero(self, dt: float):
        k = self.cfg.brake_per_s * dt
        self.vx *= max(0.0, 1.0 - k)
        self.vy *= max(0.0, 1.0 - k)
        self.vz *= max(0.0, 1.0 - k)
        self.yaw *= max(0.0, 1.0 - k)

        if abs(self.vx) < 1e-3: self.vx = 0.0
        if abs(self.vy) < 1e-3: self.vy = 0.0
        if abs(self.vz) < 1e-3: self.vz = 0.0
        if abs(self.yaw) < 1e-3: self.yaw = 0.0

    def _brake_missing_axes(self, dt: float):
        k = self.cfg.brake_per_s * dt

        if not (("W" in self.keys_down) or ("S" in self.keys_down)):
            self.vx *= max(0.0, 1.0 - k)
            if abs(self.vx) < 1e-3: self.vx = 0.0

        if not (("Q" in self.keys_down) or ("E" in self.keys_down)):
            self.vy *= max(0.0, 1.0 - k)
            if abs(self.vy) < 1e-3: self.vy = 0.0

        if not (("R" in self.keys_down) or ("F" in self.keys_down)):
            self.vz *= max(0.0, 1.0 - k)
            if abs(self.vz) < 1e-3: self.vz = 0.0

        if not (("A" in self.keys_down) or ("D" in self.keys_down)):
            self.yaw *= max(0.0, 1.0 - k)
            if abs(self.yaw) < 1e-3: self.yaw = 0.0

    # ---- AUTO follow: keep target centered + approach to distance ----
    def _auto_follow_step(self):
        if self.selected_idx < 0 or self.selected_idx >= len(self.yolo_dets):
            self.ros.publish_cmd_vel(0, 0, 0, 0)
            return

        (bgr, img_age, img_src, frame_seq,
         nearest_dist, nearest_xyz, nearest_age,
         cmd_in, cmd_in_age,
         radar, radar_age) = self.ros.get_latest()

        if bgr is None:
            self.ros.publish_cmd_vel(0, 0, 0, 0)
            return

        h, w = bgr.shape[:2]
        target = self.yolo_dets[self.selected_idx]
        cx = float(target.get("cx", w / 2.0))

        # 1) yaw to center (sign applied in publish_cmd_vel)
        e = (cx - (w / 2.0)) / (w / 2.0)  # [-1..1]
        Kp_yaw = 2.4
        yaw = clamp(-Kp_yaw * e, -2.5, 2.5)

        # 2) distance control using RADAR front if available, else nearest_dist
        dist_front = None
        if radar and radar.get("front") is not None:
            dist_front = float(radar["front"])
        elif nearest_dist is not None:
            dist_front = float(nearest_dist)

        d_ref = float(self.spin_dist.value())

        if dist_front is None:
            vx = 0.0
        else:
            Kp_v = 0.9
            vx = clamp(Kp_v * (dist_front - d_ref), -0.6, 0.9)

        # 3) slow down if target far off-center (turn first)
        off = abs(e)
        if off > 0.55:
            vx *= 0.15
        elif off > 0.35:
            vx *= 0.40
        elif off > 0.20:
            vx *= 0.70

        # 4) hard safety if too close in front
        if dist_front is not None and dist_front < max(0.35, d_ref * 0.65):
            vx = -0.35

        self.ros.publish_cmd_vel(vx, 0.0, 0.0, yaw)

    # ---- Keyboard incremental teleop ----
    def keyPressEvent(self, event: QtGui.QKeyEvent):
        k = event.key()

        if k == QtCore.Qt.Key_Space:
            self.keys_down.clear()
            self.last_input_time = now_s()
            self.follow_enabled = False
            self.all_zero()
            return

        mapping = {
            QtCore.Qt.Key_W: "W", QtCore.Qt.Key_A: "A", QtCore.Qt.Key_S: "S", QtCore.Qt.Key_D: "D",
            QtCore.Qt.Key_Q: "Q", QtCore.Qt.Key_E: "E", QtCore.Qt.Key_R: "R", QtCore.Qt.Key_F: "F",
            QtCore.Qt.Key_Shift: "SHIFT",
        }
        if k not in mapping:
            return

        key = mapping[k]
        self.keys_down.add(key)
        self.last_input_time = now_s()

        if self.mode == "AUTO":
            self.follow_enabled = False

        mult = self.cfg.turbo_mult if ("SHIFT" in self.keys_down) else 1.0

        if key == "W":
            self.vx = clamp(self.vx + self.cfg.step_lin * mult, -self.cfg.max_lin, self.cfg.max_lin)
        elif key == "S":
            self.vx = clamp(self.vx - self.cfg.step_lin * mult, -self.cfg.max_lin, self.cfg.max_lin)

        elif key == "E":
            self.vy = clamp(self.vy + self.cfg.step_lat * mult, -self.cfg.max_lat, self.cfg.max_lat)
        elif key == "Q":
            self.vy = clamp(self.vy - self.cfg.step_lat * mult, -self.cfg.max_lat, self.cfg.max_lat)

        elif key == "R":
            self.vz = clamp(self.vz + self.cfg.step_vz * mult, -self.cfg.max_vz, self.cfg.max_vz)
        elif key == "F":
            self.vz = clamp(self.vz - self.cfg.step_vz * mult, -self.cfg.max_vz, self.cfg.max_vz)

        elif key == "A":
            self.yaw = clamp(self.yaw + self.cfg.step_yaw * mult, -self.cfg.max_yaw, self.cfg.max_yaw)
        elif key == "D":
            self.yaw = clamp(self.yaw - self.cfg.step_yaw * mult, -self.cfg.max_yaw, self.cfg.max_yaw)

    def keyReleaseEvent(self, event: QtGui.QKeyEvent):
        k = event.key()
        mapping = {
            QtCore.Qt.Key_W: "W", QtCore.Qt.Key_A: "A", QtCore.Qt.Key_S: "S", QtCore.Qt.Key_D: "D",
            QtCore.Qt.Key_Q: "Q", QtCore.Qt.Key_E: "E", QtCore.Qt.Key_R: "R", QtCore.Qt.Key_F: "F",
            QtCore.Qt.Key_Shift: "SHIFT",
        }
        if k in mapping:
            self.keys_down.discard(mapping[k])
            self.last_input_time = now_s()


# -------------------------
# ROS spinning thread
# -------------------------
def start_ros_thread(node: Node, stop_event: threading.Event):
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        while rclpy.ok() and not stop_event.is_set():
            exec_.spin_once(timeout_sec=0.1)
    except Exception:
        pass
    finally:
        try:
            exec_.remove_node(node)
        except Exception:
            pass


def main():
    rclpy.init()
    ros_node = TeleopRosNode()

    stop_event = threading.Event()
    ros_thread = threading.Thread(target=start_ros_thread, args=(ros_node, stop_event), daemon=True)
    ros_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    win = TeleopWindow(ros_node, TeleopConfig())
    win.show()

    def _sigint_handler(sig, frame):
        QtCore.QTimer.singleShot(0, app.quit)

    signal.signal(signal.SIGINT, _sigint_handler)

    # keep Qt responsive to signals
    sig_timer = QtCore.QTimer()
    sig_timer.timeout.connect(lambda: None)
    sig_timer.start(200)

    code = 0
    try:
        code = app.exec()
    finally:
        # Make sure robot stops + relocate false
        try:
            win._stop_recover()
        except Exception:
            pass
        try:
            ros_node.publish_relocate(False)
        except Exception:
            pass

        stop_event.set()

        try:
            ros_node.publish_cmd_vel(0, 0, 0, 0)
            ros_node.set_mode("HOLD")
        except Exception:
            pass

        try:
            win.close()
        except Exception:
            pass

        try:
            ros_node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        try:
            ros_thread.join(timeout=2.0)
        except Exception:
            pass

    sys.exit(code)

if __name__ == "__main__":
    main()
