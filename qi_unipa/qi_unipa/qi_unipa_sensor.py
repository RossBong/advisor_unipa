import qi
import rclpy
import argparse
from rclpy.node import Node
import sys
from std_msgs.msg import String
from qi_unipa_msgs.msg import Sonar, Bumper
import time

class QiUnipa_sensor(Node):
    
    def __init__(self):
        super().__init__('qi_unipa_sensor')
         # Ottieni i parametri
        self.declare_parameter('ip', '192.168.0.161')
        self.declare_parameter('port', 9559)
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        # Connessione sessione
        self.session = self.set_connection(ip, port)
        
        self.sonar_pub = self.create_publisher(Sonar, "/sonar", 10)
        self.bumper_pub = self.create_publisher(Bumper, "/bumper", 10)
        self.speak_pub = self.create_publisher(String, "/speak", 10)
        self.pressed=False

        self.sonar_service= self.session.service("ALSonar")
        self.memory_service= self.session.service("ALMemory")

        self.timer = self.create_timer(1.0, self.get_sonar)
        self.timer = self.create_timer(0.3, self.get_bumper)

        
    def set_connection(self, ip, port):
        session = qi.Session()
        try:
            session.connect(f"tcp://{ip}:{port}")
        except RuntimeError:
            self.get_logger().error(f"Can't connect to Naoqi at ip \"{ip}\" on port {port}.\n"
                                    "Please check your script arguments.")
            sys.exit(1)
        return session
    
    def get_sonar(self):
        self.sonar_service.subscribe("Sonar_app")
        msg=Sonar()
        msg.front_sonar=self.memory_service.getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value")
        msg.back_sonar=self.memory_service.getData("Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value")
        self.sonar_pub.publish(msg)
        subscribers = self.sonar_service.getSubscribersInfo()
        if any("Sonar_app" in sub[0] for sub in subscribers):
            self.sonar_service.unsubscribe("Sonar_app")
        
    
    def get_bumper(self):
        msg=Bumper()
        msg.left=self.memory_service.getData("Device/SubDeviceList/Platform/FrontLeft/Bumper/Sensor/Value")
        msg.right=self.memory_service.getData("Device/SubDeviceList/Platform/FrontRight/Bumper/Sensor/Value")
        msg.back=self.memory_service.getData("Device/SubDeviceList/Platform/Back/Bumper/Sensor/Value")
        self.bumper_pub.publish(msg)

        if not self.pressed:
            string=String()
            if(msg.left==1.0):
                string.data="ho urtato a sinistra"
                self.speak_pub.publish(string)
            if(msg.right==1.0):
                string.data="ho urtato a destra"
                self.speak_pub.publish(string)
            if(msg.back==1.0):
                string.data="ho urtato dietro"
                self.speak_pub.publish(string)
            self.pressed = True

        if(msg.left==0.0 and msg.right==0.0 and msg.back==0.0 and self.pressed):
             self.pressed = False


def main(args=None):
    rclpy.init(args=args)
 
    node = QiUnipa_sensor()
    
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()