import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


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

        # Total number of objects that have ever entered ROI
        self.total_counter = 0

        # Number of objects currently detected inside the ROI
        self.current_objects = 0

        # Previous stable count
        self.previous_objects = 0

        self.get_logger().info('Enhanced object detection for multiple objects started.')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        frame_height, frame_width = frame.shape[:2]

        # -----------------------------
        # ROI definition
        # -----------------------------
        roi_x1 = int(frame_width * 0.42)
        roi_x2 = int(frame_width * 0.58)
        roi_y1 = int(frame_height * 0.75)
        roi_y2 = int(frame_height * 0.90)

        roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]

        # -----------------------------
        # Grayscale preprocessing
        # -----------------------------
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)

        # -----------------------------
        # 1) Threshold detection
        # -----------------------------
        _, thresh_roi = cv2.threshold(
            blurred_roi,
            100,
            255,
            cv2.THRESH_BINARY
        )

        # -----------------------------
        # 2) Edge detection
        # -----------------------------
        edges_roi = cv2.Canny(blurred_roi, 30, 100)

        # -----------------------------
        # 3) Fusion
        # -----------------------------
        fused_roi = cv2.bitwise_or(thresh_roi, edges_roi)

        # -----------------------------
        # Morphological cleanup
        # -----------------------------
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        fused_roi = cv2.morphologyEx(fused_roi, cv2.MORPH_CLOSE, kernel)
        fused_roi = cv2.morphologyEx(fused_roi, cv2.MORPH_OPEN, kernel)

        # Small erosion to help separate touching objects
        separate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        fused_roi = cv2.erode(fused_roi, separate_kernel, iterations=1)

        # -----------------------------
        # Find contours on fused result
        # -----------------------------
        contours, _ = cv2.findContours(
            fused_roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detected_objects_in_roi = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # Ignore very small noisy contours
            if area < 60:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            # Ignore too tiny shapes
            if w < 8 or h < 8:
                continue

            # Draw bounding boxes on original frame
            cv2.rectangle(
                frame,
                (roi_x1 + x, roi_y1 + y),
                (roi_x1 + x + w, roi_y1 + y + h),
                (0, 255, 0),
                2
            )

            detected_objects_in_roi += 1

        # -----------------------------
        # Current object count in ROI
        # -----------------------------
        self.current_objects = detected_objects_in_roi

        # -----------------------------
        # New counting logic
        # -----------------------------
        # If number of objects increases, add the difference
        if self.current_objects > self.previous_objects:
            new_arrivals = self.current_objects - self.previous_objects
            self.total_counter += new_arrivals
            self.get_logger().info(
                f'{new_arrivals} new object(s) detected. Total counter: {self.total_counter}'
            )

        # Update previous count for next frame
        self.previous_objects = self.current_objects

        # -----------------------------
        # Drawing
        # -----------------------------
        cv2.rectangle(
            frame,
            (roi_x1, roi_y1),
            (roi_x2, roi_y2),
            (0, 0, 255),
            2
        )

        cv2.putText(
            frame,
            'ROI',
            (roi_x1, roi_y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

        cv2.putText(
            frame,
            f'Current objects: {self.current_objects}',
            (20, 340),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (175, 214, 17),
            2
        )

        cv2.putText(
            frame,
            f'Total counter: {self.total_counter}',
            (20, 300),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (201, 53, 8),
            2
        )

        # -----------------------------
        # Debug windows
        # -----------------------------
        cv2.namedWindow("ROI Threshold", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Threshold", 700, 500)

        cv2.namedWindow("ROI Edges", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Edges", 700, 500)

        cv2.namedWindow("ROI Fused", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Fused", 700, 500)

        cv2.namedWindow("Factory Object Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Factory Object Detection", 1200, 800)

        cv2.imshow("ROI Threshold", thresh_roi)
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
