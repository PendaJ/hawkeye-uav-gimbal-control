"""ROS 2 Humble CPU-only YOLO26n + ByteTrack person tracker for Hawkeye X3.

ROS input: /drone/camera/image_raw (sensor_msgs/Image)
ROS outputs: /target_coord (geometry_msgs/Point), /tracking/image (Image)
  x = (person_center_x - image_width / 2), positive right, pixels
  y = (image_height / 2 - person_center_y), positive up, pixels
  z = 1.0 if person acquired, 0.0 otherwise
"""
import os
import threading
import time

import cv2
import rclpy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


class PersonTracker(Node):
    def __init__(self):
        super().__init__("hawkeye_person_tracker")
        self.declare_parameter("image_topic", "/drone/camera/image_raw")
        self.declare_parameter("weights", os.path.expanduser("~/hawkeye_x3_ws/yolo26n.pt"))
        self.declare_parameter("imgsz", 416)
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("cpu_threads", 4)
        self.declare_parameter("device", "cpu")  # HAWKEYE_YOLO_DEVICE_CONFIG

        image_topic = self.get_parameter("image_topic").value
        weights = os.path.expanduser(self.get_parameter("weights").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.confidence = float(self.get_parameter("confidence").value)
        threads = max(1, int(self.get_parameter("cpu_threads").value))
        # Select CPU or CUDA explicitly; never silently fall back during a GPU run.
        self.device = str(self.get_parameter("device").value).strip().lower()
        if self.device not in ("cpu", "cuda:0"):
            raise ValueError(f"Unsupported YOLO device {self.device!r}: use cpu or cuda:0")
        if self.device == "cuda:0" and not torch.cuda.is_available():
            raise RuntimeError("YOLO GPU requested, but PyTorch CUDA is unavailable")
        torch.set_num_threads(threads)
        cv2.setNumThreads(1)

        self.bridge = CvBridge()
        self.model = YOLO(weights)
        self.pub_target = self.create_publisher(Point, "/target_coord", 10)
        self.pub_image = self.create_publisher(Image, "/tracking/image", 1)
        self.sub = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )

        self._lock = threading.Lock()
        self._new_frame = threading.Event()
        self._stop = threading.Event()
        self._latest = None
        self._tracked_id = None
        self._missed = 0
        self._processed = 0
        self._last_log = time.monotonic()
        self._last_processed = self._last_log
        self._last_target_publish = self._last_log
        self._last_image_callback = self._last_log
        # HAWKEYE_VISION_STALL_DIAGNOSTIC: timing only; no change to tracking logic.
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"{self.device.upper()} YOLO+ByteTrack started: {weights}, imgsz={self.imgsz}, "
            f"threads={threads}, camera={image_topic}. "
            "Displaying annotated images only when /tracking/image is subscribed."
        )

    def _on_image(self, msg):
        callback_started = time.monotonic()
        self._last_image_callback = callback_started
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Camera image conversion failed: {exc}")
            return
        conversion_ms = 1000.0 * (time.monotonic() - callback_started)
        if conversion_ms > 250.0:
            self.get_logger().warning(
                f"HAWKEYE_IMAGE_CALLBACK_SLOW conversion_ms={conversion_ms:.1f}"
            )
        # Replace, never queue: the detector always sees the most recent frame.
        with self._lock:
            self._latest = (frame, msg.header)
            self._new_frame.set()

    def _worker_loop(self):
        while rclpy.ok() and not self._stop.is_set():
            if not self._new_frame.wait(timeout=0.2):
                continue
            with self._lock:
                sample = self._latest
                self._latest = None
                self._new_frame.clear()
            if sample is None:
                continue
            frame, header = sample
            started = time.monotonic()
            try:
                results = self.model.track(
                    frame,
                    persist=True,
                    tracker="bytetrack.yaml",
                    device=self.device,
                    imgsz=self.imgsz,
                    conf=self.confidence,
                    classes=[0],
                    max_det=8,
                    verbose=False,
                )[0]
                inference_end = time.monotonic()
                self._publish_results(frame, header, results)
                publish_end = time.monotonic()
                gap_ms = 1000.0 * (publish_end - self._last_target_publish)
                if gap_ms > 450.0:
                    infer_ms = 1000.0 * (inference_end - started)
                    publish_ms = 1000.0 * (publish_end - inference_end)
                    callback_age_ms = 1000.0 * (publish_end - self._last_image_callback)
                    self.get_logger().warning(
                        f"HAWKEYE_VISION_STALL gap_ms={gap_ms:.1f} "
                        f"inference_ms={infer_ms:.1f} "
                        f"publish_ms={publish_ms:.1f} "
                        f"latest_camera_callback_age_ms={callback_age_ms:.1f}"
                    )
                self._last_target_publish = publish_end
            except Exception as exc:
                self.get_logger().error(f"YOLO tracking failed: {exc}")
                self.pub_target.publish(Point(x=0.0, y=0.0, z=0.0))
                time.sleep(0.1)
                continue

            self._processed += 1
            now = time.monotonic()
            if now - self._last_log >= 5.0:
                elapsed = now - self._last_log
                self.get_logger().info(
                    f"Processed {self._processed / elapsed:.1f} frames/s ({self.device.upper()}), "
                    f"latest inference {1000 * (now - started):.1f} ms"
                )
                self._last_log = now
                self._processed = 0

    def _publish_results(self, frame, header, results):
        boxes = results.boxes
        chosen = None
        person_boxes = []
        if boxes is not None and len(boxes) > 0:
            coords = boxes.xyxy.cpu().numpy()
            ids = boxes.id.cpu().numpy().astype(int) if boxes.id is not None else None
            confs = boxes.conf.cpu().numpy()
            for i, xyxy in enumerate(coords):
                person_boxes.append({
                    "xyxy": xyxy,
                    "id": int(ids[i]) if ids is not None else None,
                    "conf": float(confs[i]),
                    "area": float((xyxy[2]-xyxy[0]) * (xyxy[3]-xyxy[1])),
                })

        if person_boxes:
            if self._tracked_id is not None:
                chosen = next((b for b in person_boxes if b["id"] == self._tracked_id), None)
            if chosen is not None:
                self._missed = 0
            elif self._tracked_id is None or all(b["id"] is None for b in person_boxes):
                chosen = max(person_boxes, key=lambda b: b["area"])
                self._tracked_id = chosen["id"]
                self._missed = 0
            else:
                self._missed += 1
                if self._missed >= 3:
                    chosen = max(person_boxes, key=lambda b: b["area"])
                    self._tracked_id = chosen["id"]
                    self._missed = 0
        else:
            self._missed += 1
            if self._missed >= 3:
                self._tracked_id = None

        h, w = frame.shape[:2]
        if chosen is None:
            target = Point(x=0.0, y=0.0, z=0.0)
        else:
            x1, y1, x2, y2 = chosen["xyxy"]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            target = Point(
                x=float(cx - w / 2.0),
                y=float(h / 2.0 - cy),
                z=1.0,
            )
        self.pub_target.publish(target)

        # Avoid expensive overlays + ROS image transmission unless someone wants to view them.
        if self.pub_image.get_subscription_count() > 0:
            vis = frame.copy()
            cv2.drawMarker(vis, (w // 2, h // 2), (255, 255, 0), cv2.MARKER_CROSS, 18, 2)
            for b in person_boxes:
                x1, y1, x2, y2 = map(int, b["xyxy"])
                active = b is chosen
                color = (0, 220, 0) if active else (0, 165, 255)
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                text = f"person ID {b['id']} {b['conf']:.2f}"
                cv2.putText(vis, text, (x1, max(15, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
            status = (f"TRACKING dx={target.x:+.0f} dy={target.y:+.0f}"
                      if target.z > 0.5 else "NO PERSON")
            cv2.putText(vis, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 220, 0) if target.z > 0.5 else (0, 0, 255), 2)
            out = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            out.header = header
            self.pub_image.publish(out)

    def stop(self):
        self._stop.set()
        self._new_frame.set()
        self._worker.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
