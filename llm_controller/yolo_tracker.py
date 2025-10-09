#!/usr/bin/env python3
"""
YOLO Person Tracker - Detects and tracks people in real-time
Publishes detections for VLM to consume
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class YOLOTracker(Node):
    def __init__(self):
        super().__init__('yolo_tracker')
        
        # Load YOLOv11-small
        print("Loading YOLOv11-small...")
        self.model = YOLO('yolo11s.pt')  # Auto-downloads on first run
        
        # QoS matching Isaac Sim camera (RELIABLE)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to camera
        self.bridge = CvBridge()
        self.create_subscription(
            Image,
            '/drone1/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )
        
        # Publish detections as JSON
        self.detection_pub = self.create_publisher(
            String,
            '/drone1/yolo_detections',
            10
        )

        self.annotated_pub = self.create_publisher(
            Image, 
            '/drone1/yolo_annotated', 
            10
        )
        
        # Store latest detections (for VLM to read)
        self.latest_detections = []
        
        print("✅ YOLO Tracker ready!")
        
    def image_callback(self, msg):
        """Run YOLO on each frame"""
        # Convert ROS image to numpy
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        
        # Run YOLO tracking (maintains IDs across frames)
        results = self.model.track(
            cv_image,
            persist=True,      # Keep track IDs
            classes=[0],       # Class 0 = person
            conf=0.5,          # 50% confidence threshold
            verbose=False
        )

        annotated = results[0].plot()

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
        annotated_msg.header.frame_id = 'camera'
        self.annotated_pub.publish(annotated_msg)

        # Extract detections
        detections = []
        if results[0].boxes is not None:
            boxes = results[0].boxes
            
            for i in range(len(boxes)):
                box = boxes[i]
                
                # Get bounding box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # Get tracking ID (if available)
                track_id = int(box.id[0]) if box.id is not None else -1
                
                # Get confidence
                conf = float(box.conf[0])
                
                detection = {
                    'id': track_id,
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'center': [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                    'confidence': round(conf, 2),
                    'class': 'person'
                }
                detections.append(detection)
        
        # Store and publish
        self.latest_detections = detections
        
        # Publish as JSON
        msg = String()
        msg.data = json.dumps({'detections': detections, 'count': len(detections)})
        self.detection_pub.publish(msg)
        
    def get_detections(self):
        """Called by VLM to get current detections"""
        return self.latest_detections

def main():
    rclpy.init()
    tracker = YOLOTracker()
    rclpy.spin(tracker)

if __name__ == "__main__":
    main()
