#!/usr/bin/env python3
"""ROS1 OBB vision adapter. Preview topics by default; no robot commands."""
import json
import math
import os
import threading

import numpy as np


def obb_geometry(corners):
    corners = np.asarray(corners, dtype=float).reshape(4, 2)
    if not np.isfinite(corners).all():
        raise ValueError("Non-finite OBB coordinates")
    edges = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(edges, axis=1)
    if lengths.max() <= 0:
        raise ValueError("Degenerate OBB")
    edge = edges[int(np.argmax(lengths))]
    # Undirected long axis, degrees in image coordinates (x right, y down).
    # This is NOT a robot wrist angle or a full 3D orientation.
    yaw = (math.degrees(math.atan2(edge[1], edge[0])) + 90) % 180 - 90
    return corners.mean(axis=0), float(yaw)


def depth_at_center(depth, u, v, scale):
    if scale <= 0 or not math.isfinite(scale):
        raise ValueError("Depth scale must be finite and positive")
    x, y = int(round(u)), int(round(v))
    if not (0 <= y < depth.shape[0] and 0 <= x < depth.shape[1]):
        return None
    value = float(depth[y, x]) * scale
    return value if math.isfinite(value) and value > 0 else None


def camera_point(u, v, z, k):
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if not all(math.isfinite(x) for x in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError("Invalid camera intrinsics")
    return [(u - cx) * z / fx, (v - cy) * z / fy, z]


class VisionNode:
    def __init__(self):
        import cv2
        import message_filters
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String
        from ultralytics import YOLO

        self.ros, self.cv2, self.String = rospy, cv2, String
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.info = None
        path = os.path.expanduser(rospy.get_param("~model", "~/jaka/robot1_vision.pt"))
        if not os.path.isfile(path):
            raise RuntimeError("Model file not found: " + path)
        self.model = YOLO(path)
        if self.model.task != "obb":
            raise RuntimeError("This adapter requires an OBB model")
        self.conf = float(rospy.get_param("~conf", 0.35))
        self.imgsz = int(rospy.get_param("~imgsz", 768))
        self.show = bool(rospy.get_param("~show", True))
        self.scale = rospy.get_param("~depth_scale", None)
        # Equal resolution is NOT proof of depth/RGB pixel registration.
        self.aligned = bool(rospy.get_param("~depth_aligned", False))
        self.official = bool(rospy.get_param("~publish_official", False))
        self.names = rospy.get_param("~class_map", {})
        self.json_pub = rospy.Publisher("/robot1_vision/detections", String, queue_size=1)
        self.image_pub = rospy.Publisher("/robot1_vision/annotated_image", Image, queue_size=1)
        self.pose_pub = self.official_image_pub = None
        if self.official:
            from large_scale_model_arm.msg import Mycaryolo
            self.Mycaryolo = Mycaryolo
            self.pose_pub = rospy.Publisher("mycaryolo", Mycaryolo, queue_size=10)
            self.official_image_pub = rospy.Publisher("yolo_annotated_image", Image, queue_size=1)
            rospy.logwarn("Official image output enabled: stop old detector and robot control before testing")
        self.info_sub = rospy.Subscriber(
            rospy.get_param("~info_topic", "/Jaka/camera/camera_info"),
            CameraInfo, self.on_info, queue_size=1)
        self.rgb_sub = message_filters.Subscriber(
            rospy.get_param("~rgb_topic", "/Jaka/camera/rgb"), Image, queue_size=1)
        self.depth_sub = message_filters.Subscriber(
            rospy.get_param("~depth_topic", "/Jaka/camera/depth"), Image, queue_size=1)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5,
            slop=float(rospy.get_param("~sync_slop", 0.1)))
        self.sync.registerCallback(self.on_images)
        rospy.loginfo("OBB model loaded: %s; classes=%s", path, self.model.names)
        rospy.loginfo("Preview only=%s; 3D enabled after alignment confirmation=%s",
                      not self.official, self.aligned)

    def on_info(self, msg):
        self.info = msg

    def on_images(self, rgb_msg, depth_msg):
        if not self.lock.acquire(False):
            return
        try:
            self.process(rgb_msg, depth_msg)
        except Exception as exc:
            self.ros.logerr_throttle(5, "Vision frame failed: %s" % exc)
        finally:
            self.lock.release()

    def process(self, rgb_msg, depth_msg):
        cv2 = self.cv2
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        canvas = rgb.copy()
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        scale = self.scale
        if scale is None and depth_msg.encoding == "32FC1":
            scale = 1.0  # Official supplied script and documentation use metres.
        if depth_msg.encoding not in ("32FC1", "16UC1"):
            scale = None
        info = self.info
        use_depth = self.aligned and scale is not None and depth.shape == rgb.shape[:2]
        use_camera = (use_depth and info is not None and
                      (info.height, info.width) == rgb.shape[:2] and
                      info.header.frame_id == rgb_msg.header.frame_id)
        if not use_camera:
            self.ros.logwarn_throttle(15, "2D preview: confirm depth registration, scale, camera_info size/frame before 3D")
        result = self.model.predict(rgb, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        items = []
        if result.obb is not None:
            for corners, cid, confidence in zip(
                    result.obb.xyxyxyxy.cpu().numpy(),
                    result.obb.cls.cpu().numpy(), result.obb.conf.cpu().numpy()):
                center, angle = obb_geometry(corners)
                u, v = map(float, center)
                original = result.names[int(cid)]
                name = self.names.get(original, original)
                z = depth_at_center(depth, u, v, float(scale)) if use_depth else None
                position = camera_point(u, v, z, info.K) if use_camera and z is not None else None
                items.append(dict(class_id=int(cid), model_class=original, name=name,
                                  confidence=float(confidence), center_pixel=[u, v],
                                  yaw_image_deg=angle, corners_pixel=corners.tolist(),
                                  depth_m=z, position_camera_m=position))
                cv2.polylines(canvas, [np.rint(corners).astype(np.int32)], True, (0, 255, 0), 2)
                ztext = "%.2fm" % z if z is not None else "depth=N/A"
                label = "%s (%d,%d,%s,%.2f)" % (name, round(u), round(v), ztext, angle)
                cv2.putText(canvas, label, (max(0, int(corners[:, 0].min())),
                            max(20, int(corners[:, 1].min()) - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                if self.pose_pub is not None and position is not None:
                    msg = self.Mycaryolo(conf=float(confidence), name=name)
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
                    # Match official position-only contract; do not fabricate a 3D orientation.
                    self.pose_pub.publish(msg)
        payload = dict(stamp=rgb_msg.header.stamp.to_sec(),
                       frame_id=rgb_msg.header.frame_id, detections=items,
                       angle_convention="undirected image long axis, degrees [-90,90)")
        self.json_pub.publish(self.String(data=json.dumps(payload, allow_nan=False)))
        image_msg = self.bridge.cv2_to_imgmsg(canvas, "bgr8")
        image_msg.header = rgb_msg.header
        self.image_pub.publish(image_msg)
        if self.official_image_pub is not None:
            self.official_image_pub.publish(image_msg)
        if self.show:
            cv2.namedWindow("Robot1 OBB Preview", cv2.WINDOW_NORMAL)
            cv2.imshow("Robot1 OBB Preview", canvas)
            cv2.waitKey(1)


def main():
    import rospy
    import cv2
    rospy.init_node("robot1_obb_vision", anonymous=True)
    node = VisionNode()
    try:
        rospy.spin()
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
