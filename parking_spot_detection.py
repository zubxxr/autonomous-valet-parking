import cv2
import pandas as pd
import numpy as np
from ultralytics import YOLO
import time
import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

model = YOLO('weights/rosmaster_yolov8.pt')

cv2.namedWindow('RGB')

my_file = open("coco.txt", "r")
data = my_file.read()
class_list = data.split("\n")

# = tl, tr, br bl
area1 = [(310, 188), (438, 192), (434, 297), (290, 311)]

area2 = [(442, 190), (571, 193), (575, 315), (434, 296)]

area3 = [(571, 191), (700, 194), (715, 315), (577, 315)]

area4 = [(703, 195), (817, 196), (845, 311), (714, 315)]


# Define a class for the ROS node
class ParkingSpotDetectorNode(Node):
    def __init__(self):
        super().__init__('parking_spot_detector')
        self.publisher_ = self.create_publisher(String, 'available_parking_spots', 10)

    def publish_priority_queue(self, priority_queue):
        msg = String()
        msg.data = f"[{', '.join(priority_queue)}]"
        self.publisher_.publish(msg)

def detect_and_publish_frames(node, cap):
    start_time = time.time()
    frame_counter = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (1020, 500))

        results = model.predict(frame)
        a = results[0].boxes.data.cpu().numpy()
        px = pd.DataFrame(a).astype("float")

        list1 = []
        list2 = []
        list3 = []
        list4 = []
        priority_queue = []

        for index, row in px.iterrows():
            x1 = int(row[0])
            y1 = int(row[1])
            x2 = int(row[2])
            y2 = int(row[3])
            d = int(row[5])
            c = class_list[d]
            if 'Rosmasters' in c:
                cx = int(x1 + x2) // 2
                cy = int(y1 + y2) // 2

                results1 = cv2.pointPolygonTest(np.array(area1, np.int32), ((cx, cy)), False)
                if results1 >= 0:
                    list1.append(c)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                    cv2.putText(frame, str(c), (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)

                results2 = cv2.pointPolygonTest(np.array(area2, np.int32), ((cx, cy)), False)
                if results2 >= 0:
                    list2.append(c)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                    cv2.putText(frame, str(c), (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)

                results3 = cv2.pointPolygonTest(np.array(area3, np.int32), ((cx, cy)), False)
                if results3 >= 0:
                    list3.append(c)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                    cv2.putText(frame, str(c), (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)

                results4 = cv2.pointPolygonTest(np.array(area4, np.int32), ((cx, cy)), False)
                if results4 >= 0:
                    list4.append(c)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                    cv2.putText(frame, str(c), (x1, y1), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)

        a1 = len(list1)
        a2 = len(list2)
        a3 = len(list3)
        a4 = len(list4)

        o = a1 + a2 + a3 + a4
        space = 4 - o
        
        # if a = 1, parking spot is occupied; else, parking spot is free
        if a1 == 1:
            # Draw the red text with thickness=2
            cv2.putText(frame, str('1'), (380, 441), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 255), 2)
            # Draw the white text with thickness=1 (background color)
            cv2.putText(frame, str('1'), (380 - 1, 441), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 1)
        else:
            # Draw the white text with thickness=2 (background color)
            cv2.putText(frame, str('1'), (380, 441), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 2)

        if a2 == 1:
            cv2.putText(frame, str('2'), (500, 440), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 255), 2)
            cv2.putText(frame, str('2'), (500 - 1, 440), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 1)
        else:
            cv2.putText(frame, str('2'), (500, 440), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 2)

        if a3 == 1:
            cv2.putText(frame, str('3'), (600, 436), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 255), 2)
            cv2.putText(frame, str('3'), (600 - 1, 436), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 1)
        else:
            cv2.putText(frame, str('3'), (600, 436), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 2)

        if a4 == 1:
            cv2.putText(frame, str('4'), (700, 436), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 255), 2)
            cv2.putText(frame, str('4'), (700 - 1, 436), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 1)
        else:
            cv2.putText(frame, str('4'), (700, 436), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 2)



        priority_dict = {'a1': a1, 'a2': a2, 'a3': a3, 'a4': a4}
        sorted_priority = sorted(priority_dict.items(), key=lambda x: int(x[0][1:]))

        for key, value in sorted_priority:
            if value != 1:
                priority_queue.append(key)

        print("Priority Queue:", priority_queue)

        for item in priority_queue:
            cv2.putText(frame, item, (50, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255), 2)

        cv2.putText(frame, str(space), (23, 30), cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255), 2)

        cv2.imshow("RGB", frame)

        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

        # Publish priority queue to ROS 2 topic every second
        elapsed_time = time.time() - start_time
        if elapsed_time > frame_counter * 1:  # Adjust this value depending on your FPS
            node.publish_priority_queue(priority_queue)
            frame_counter += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    rclpy.init()

    node = ParkingSpotDetectorNode()

    cap = cv2.VideoCapture("videos/2024-03-05-134820.webm")
    detect_and_publish_frames(node, cap)

    node.destroy_node()
    rclpy.shutdown()
