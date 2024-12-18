import qi
import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np

# Costanti per la risoluzione (prova modifica)
K_QVGA = 1     # 320x240
K_VGA = 2      # 640x480
K_HD = 4       # 1280x720
# Costanti per lo spazio colore
K_RGB = 13
K_BGR = 11

class QiUnipa_vision(Node):
    
    def __init__(self):
        super().__init__('qi_unipa_vision')
        # Inizializza CvBridge all'interno della classe
        self.bridge = CvBridge()
        
        # Ottieni i parametri
        self.declare_parameter('ip', '192.168.0.161')
        self.declare_parameter('port', 9559)
        self.declare_parameter('camera_index', 0)#0 Top Camera, 1 Bottom Camera
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value

        
        # Connessione sessione
        self.session = self.set_connection(ip, port)
        
        self.camera_pub = self.create_publisher(Image, '/camera', 10)
        self.video_service = self.session.service("ALVideoDevice")

        self.camera_timer = self.create_timer(0.1, self.get_camera)
        
    def set_connection(self, ip, port):
        session = qi.Session()
        try:
            session.connect(f"tcp://{ip}:{port}")
        except RuntimeError:
            self.get_logger().error(f"Can't connect to Naoqi at ip \"{ip}\" on port {port}.\n"
                                    "Please check your script arguments.")
            sys.exit(1)
        return session
    
    def get_camera(self):
        self.video_client = self.video_service.subscribeCamera("Camera", self.camera_index, K_VGA, K_BGR, 30)
        result = self.video_service.getImageRemote(self.video_client)
        
        if result is None:
            print("Cannot get image")
            return
                   
        width = result[0]
        height = result[1]
        channels = result[2]
        image_binary = result[6]
        
        # Converti l'immagine binaria in un array numpy
        image_array = np.frombuffer(image_binary, dtype=np.uint8)
        image_array = image_array.reshape((height, width, channels))
        
        # Pubblica l'immagine
        try:
            img_msg = self.bridge.cv2_to_imgmsg(image_array, encoding="bgr8")
            self.camera_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')
        finally:
            self.video_service.unsubscribe(self.video_client)
        
def main(args=None):
    rclpy.init(args=args)
    node = QiUnipa_vision()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()