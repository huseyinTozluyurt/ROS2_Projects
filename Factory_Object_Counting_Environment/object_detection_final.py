import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

# Object Detection
# Detection of Multiple Objects at the same time
# Detection even though to the shadow in the simulation
# Corrected Counters for 'Current Objects' and 'Total Counter'
class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image',
            self.image_callback,
            10
        )

        # Counters
        self.total_counter = 0
        self.current_objects = 0

        # Background subtractor for shadow-robust foreground extraction
        # detectShadows=False avoids gray shadow labels that often confuse counting
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500,
            varThreshold=28,
            detectShadows=False
        )

        # Simple centroid tracker state
        self.next_track_id = 0
        self.tracks = {}  # track_id -> dict(cx, cy, missed, counted)

        # Counting line inside ROI
        self.count_line_ratio = 0.55

        # Warmup frames so the background model can stabilize
        self.frame_count = 0
        self.warmup_frames = 20

        self.get_logger().info(
            'Shadow-robust object detection with centroid line-cross counting started.'
        )

    def _get_roi(self, frame):
        frame_height, frame_width = frame.shape[:2]

        roi_x1 = int(frame_width * 0.42)
        roi_x2 = int(frame_width * 0.58)
        roi_y1 = int(frame_height * 0.75)
        roi_y2 = int(frame_height * 0.90)

        return roi_x1, roi_y1, roi_x2, roi_y2, frame[roi_y1:roi_y2, roi_x1:roi_x2]

    def _preprocess_roi(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # 1) Background subtraction
        fg_mask = self.bg_subtractor.apply(blur)

        # 2) Adaptive threshold for local illumination/shadow robustness
        adaptive = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            21,
            5
        )

        # 3) Mild edge extraction
        edges = cv2.Canny(blur, 35, 100)

        # Reduce pure shadow/edge noise by giving more weight to foreground mask
        fused = cv2.bitwise_and(adaptive, fg_mask)
        fused = cv2.bitwise_or(fused, edges)

        # Morphology
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        fused = cv2.morphologyEx(fused, cv2.MORPH_CLOSE, kernel_close)
        fused = cv2.morphologyEx(fused, cv2.MORPH_OPEN, kernel_open)

        # Small dilation to restore weak dark-object regions
        fused = cv2.dilate(fused, kernel_open, iterations=1)

        return gray, blur, fg_mask, adaptive, edges, fused

    def _extract_detections(self, fused_roi):
        contours, _ = cv2.findContours(
            fused_roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detections = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 90:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            if w < 12 or h < 12:
                continue

            rect_area = w * h
            fill_ratio = area / float(rect_area) if rect_area > 0 else 0.0

            # Reject thin or weird fragments often caused by shadows/edges
            if fill_ratio < 0.22:
                continue

            aspect_ratio = w / float(h)
            if aspect_ratio < 0.20 or aspect_ratio > 5.5:
                continue

            cx = x + w // 2
            cy = y + h // 2

            detections.append({
                'bbox': (x, y, w, h),
                'centroid': (cx, cy),
                'area': area
            })

        return detections

    def _match_tracks(self, detections):
        """
        Greedy centroid matching.
        Good enough here because object count is small.
        """
        max_distance = 45

        unmatched_detection_indices = set(range(len(detections)))
        updated_tracks = {}

        existing_ids = list(self.tracks.keys())
        used_track_ids = set()

        for det_idx, det in enumerate(detections):
            dcx, dcy = det['centroid']

            best_track_id = None
            best_distance = float('inf')

            for track_id in existing_ids:
                if track_id in used_track_ids:
                    continue

                tx = self.tracks[track_id]['cx']
                ty = self.tracks[track_id]['cy']

                dist = math.hypot(dcx - tx, dcy - ty)

                if dist < best_distance and dist < max_distance:
                    best_distance = dist
                    best_track_id = track_id

            if best_track_id is not None:
                prev_track = self.tracks[best_track_id]
                updated_tracks[best_track_id] = {
                    'cx': dcx,
                    'cy': dcy,
                    'prev_cx': prev_track['cx'],
                    'prev_cy': prev_track['cy'],
                    'missed': 0,
                    'counted': prev_track['counted'],
                    'age': prev_track.get('age', 1) + 1
                }
                used_track_ids.add(best_track_id)
                unmatched_detection_indices.discard(det_idx)

        # Tracks not matched this frame
        for track_id in existing_ids:
            if track_id not in updated_tracks:
                prev_track = self.tracks[track_id]
                missed = prev_track['missed'] + 1
                if missed <= 6:
                    updated_tracks[track_id] = {
                        'cx': prev_track['cx'],
                        'cy': prev_track['cy'],
                        'prev_cx': prev_track.get('prev_cx', prev_track['cx']),
                        'prev_cy': prev_track.get('prev_cy', prev_track['cy']),
                        'missed': missed,
                        'counted': prev_track['counted'],
                        'age': prev_track.get('age', 1)
                    }

        # New tracks
        for det_idx in unmatched_detection_indices:
            dcx, dcy = detections[det_idx]['centroid']
            updated_tracks[self.next_track_id] = {
                'cx': dcx,
                'cy': dcy,
                'prev_cx': dcx,
                'prev_cy': dcy,
                'missed': 0,
                'counted': False,
                'age': 1
            }
            self.next_track_id += 1

        self.tracks = updated_tracks

    



#    def _update_counter(self, roi_height):
#        count_line_y = int(roi_height * self.count_line_ratio)
#
#        for track_id, track in self.tracks.items():
#            if track['missed'] > 0 or track['counted']:
#                continue
#
#           prev_y = track['prev_cy']
#            curr_y = track['cy']
#
#            # Count when object centroid crosses the line downward
#            if prev_y < count_line_y <= curr_y:
#                self.total_counter += 1
#                track['counted'] = True
#                self.get_logger().info(
#                    f'Object counted (track {track_id}). Total counter: {self.total_counter}'
#                )
#
#        return count_line_y


#    def _update_counter(self, roi_width):
#        count_line_x = int(roi_width * self.count_line_ratio)
#
#        for track_id, track in self.tracks.items():
#            if track['missed'] > 0 or track['counted']:
#                continue
#
#            prev_x = track['prev_cx']
#            curr_x = track['cx']
#
#            # Count when object centroid crosses the vertical line from left to right
#            if prev_x < count_line_x <= curr_x:
#                self.total_counter += 1
#                track['counted'] = True
#                self.get_logger().info(
#                    f'Object counted (track {track_id}). Total counter: {self.total_counter}'
#                )

#        return count_line_x


    def _update_counter(self):
        min_stable_age = 3

        for track_id, track in self.tracks.items():
            if track['missed'] > 0:
                continue

            if track['counted']:
                continue

            if track.get('age', 1) >= min_stable_age:
                self.total_counter += 1
                track['counted'] = True
                self.get_logger().info(
                    f'Object counted after stable tracking. Track ID: {track_id}, Total: {self.total_counter}'
                )





    

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_count += 1

        roi_x1, roi_y1, roi_x2, roi_y2, roi = self._get_roi(frame)
        roi_h, roi_w = roi.shape[:2]

        gray_roi, blur_roi, fg_mask, adaptive_roi, edges_roi, fused_roi = self._preprocess_roi(roi)

        # Let the background model settle before real counting starts
        if self.frame_count <= self.warmup_frames:
            count_line_x = int(roi_w * self.count_line_ratio)

            cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 0, 255), 2)

            cv2.line(frame,
                    (roi_x1 + count_line_x, roi_y1),
                    (roi_x1 + count_line_x, roi_y2),
                    (255, 0, 255), 2)

            cv2.putText(frame, 'Warming up background model...',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(frame, f'Current objects: 0',
                        (20, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (175, 214, 17), 2)
            cv2.putText(frame, f'Total counter: {self.total_counter}',
                        (20, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (201, 53, 8), 2)

            cv2.namedWindow("ROI Foreground", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("ROI Foreground", 700, 500)

            cv2.namedWindow("ROI Adaptive", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("ROI Adaptive", 700, 500)

            cv2.namedWindow("ROI Fused", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("ROI Fused", 700, 500)

            cv2.namedWindow("Factory Object Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Factory Object Detection", 1200, 800)

            cv2.imshow("ROI Foreground", fg_mask)
            cv2.imshow("ROI Adaptive", adaptive_roi)
            cv2.imshow("ROI Fused", fused_roi)
            cv2.imshow("Factory Object Detection", frame)
            cv2.waitKey(1)
            return


        detections = self._extract_detections(fused_roi)
        self._match_tracks(detections)

        self._update_counter()

        count_line_y = int(roi_h * self.count_line_ratio)

        # Count only active visible tracks
        self.current_objects = sum(
            1 for t in self.tracks.values() if t['missed'] == 0
        )

        # Draw ROI and counting line
        cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 0, 255), 2)
        cv2.putText(frame, 'ROI',
                    (roi_x1, roi_y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.line(frame,
                (roi_x1, roi_y1 + count_line_y),
                (roi_x2, roi_y1 + count_line_y),
                (255, 0, 255), 2)

        cv2.putText(frame, 'Count line',
                    (roi_x1 + 8, roi_y1 + count_line_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        # Draw detections and tracks
        for det in detections:
            x, y, w, h = det['bbox']
            cx, cy = det['centroid']

            cv2.rectangle(frame,
                          (roi_x1 + x, roi_y1 + y),
                          (roi_x1 + x + w, roi_y1 + y + h),
                          (0, 255, 0), 2)
            cv2.circle(frame, (roi_x1 + cx, roi_y1 + cy), 4, (0, 255, 255), -1)

        for track_id, track in self.tracks.items():
            if track['missed'] > 0:
                continue

            tx, ty = track['cx'], track['cy']
            cv2.putText(frame, f'ID {track_id}',
                        (roi_x1 + tx + 6, roi_y1 + ty - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

        # HUD
        cv2.putText(frame,
                    f'Current objects: {self.current_objects}',
                    (20, 340),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (175, 214, 17),
                    2)

        cv2.putText(frame,
                    f'Total counter: {self.total_counter}',
                    (20, 300),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (201, 53, 8),
                    2)

        # Debug windows
        cv2.namedWindow("ROI Foreground", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Foreground", 700, 500)

        cv2.namedWindow("ROI Adaptive", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Adaptive", 700, 500)

        cv2.namedWindow("ROI Edges", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Edges", 700, 500)

        cv2.namedWindow("ROI Fused", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Fused", 700, 500)

        cv2.namedWindow("Factory Object Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Factory Object Detection", 1200, 800)

        cv2.imshow("ROI Foreground", fg_mask)
        cv2.imshow("ROI Adaptive", adaptive_roi)
        cv2.imshow("ROI Edges", edges_roi)
        cv2.imshow("ROI Fused", fused_roi)
        cv2.imshow("Factory Object Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
