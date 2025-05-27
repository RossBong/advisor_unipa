#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os 
import cv2
import numpy as np
import matplotlib.pyplot as plt

class VideoController(Node):
    def __init__(self):
        super().__init__("vision_yolo")
        self.subscription = self.create_subscription(Image, "/camera", self.image_callback, 100)
        self.walk_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.bridge = CvBridge()

        # Specifica il percorso ai file YOLO
        yolo_dir = os.path.expanduser("/home/antonio/ros2_ws_robotics/yolo_files")
        #yolo_dir = os.path.expanduser("../../yolo_files")
        weights_path = os.path.join(yolo_dir, "yolov3.weights")
        config_path = os.path.join(yolo_dir, "yolov3.cfg")

        # Carica YOLO
        self.net = cv2.dnn.readNet(weights_path, config_path)
        with open(os.path.join(yolo_dir, "coco.names"), "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.net.getLayerNames()

        # Gestione dell'output delle layer non connesse
        output_layers_indices = self.net.getUnconnectedOutLayers()
        if len(output_layers_indices) > 0:
            self.output_layers = [self.layer_names[i - 1] for i in output_layers_indices.flatten()]
        else:
            self.output_layers = []

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
   
            # Prepara l'immagine per YOLO
            blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            self.net.setInput(blob)
            outs = self.net.forward(self.output_layers)

            # Inizializza le liste per le informazioni sugli oggetti rilevati
            class_ids = []
            confidences = []
            boxes = []

            # Elaborazione degli output di YOLO
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)

                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)

                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)

            # Applica non-maxima suppression
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

            # Disegna i bounding box
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(self.classes[class_ids[i]])
                    confidence = confidences[i]
                    
                    # Disegna il rettangolo e il testo
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"{label} {confidence:.2f}", (x, y + 30),
                              cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

            # Aspetta un tasto per chiudere la finestra
            plt.clf()
            plt.imshow(cv_image)
            plt.axis('off')
            plt.show(block=False)
            plt.pause(0.001)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    video_controller = VideoController()
    try:
        rclpy.spin(video_controller)
    except KeyboardInterrupt:
        pass
    finally:
        video_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()