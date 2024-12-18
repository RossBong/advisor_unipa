import qi
import sys
import rclpy
from rclpy.node import Node
from qi_unipa_msgs.msg import PostureWithSpeed,Track
class QiUnipatracking(Node):  
    def __init__(self):
        super().__init__('qi_unipa_tracking')
        
        # Ottieni i parametri
        self.declare_parameter('ip', '192.168.0.161')
        self.declare_parameter('port', 9559)
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        # Connessione sessione
        self.session = self.set_connection(ip, port)

        self.tracker_service = self.session.service("ALTracker")
        self.motion_service = self.session.service("ALMotion")
        self.tracker_service.setMode("Head") #"WholeBody"
        #self.tracker_service.setMoveConfig([["MaxVelXY", 0.2], ["MaxVelTheta", 0.5], ["MaxAccXY", 0.2], ["MAxAccTheta", 0.5]])

        self.tracking_sub = self.create_subscription(Track, "/track", self.start_tracking, 10)
        self.posture_pub = self.create_publisher(PostureWithSpeed, "/posture", 10)


    def set_connection(self, ip, port):
            session = qi.Session()
            try:
                session.connect(f"tcp://{ip}:{port}")
            except RuntimeError:
                self.get_logger().error(f"Can't connect to Naoqi at ip \"{ip}\" on port {port}.\n"
                                        "Please check your script arguments.")
                sys.exit(1)
            return session
    
    def pub_posture(self, name, speed):
        msg=PostureWithSpeed()
        msg.posture_name=name
        msg.speed=speed
        self.posture_pub.publish(msg)

    def start_tracking(self,msg_in):

        self.pub_posture("Stand", 0.5)
        if msg_in.target_name =="Face":
            params=msg_in.distance

        elif msg_in.target_name =="Sound":
            confidence=0.5
            params=[msg_in.distance,confidence]

        elif msg_in.target_name =="Stop":
            self.stop_tracking()
            return 
        
        elif msg_in.target_name=="People":
            params=[0]
    

        self.tracker_service.registerTarget(msg_in.target_name, params)

        self.tracker_service.setRelativePosition([-2.0, 0.0, 0.0, 0.1, 0.1, 0.3])
        #self.tracker_service.setMaximumVelocity(0.8) #Head
        #self.tracker_service.setMaximumDistanceDetection(msg_in.distance) # Oltre il target è perso
        
        self.tracker_service.track(msg_in.target_name)
        
        #a = self.tracker_service.getMoveConfig()
        #self.get_logger().info(a)


    def stop_tracking(self):
        self.tracker_service.stopTracker()
        self.tracker_service.unregisterAllTargets()
        self.pub_posture("Stand", 0.5)



def main(args=None):
    rclpy.init(args=args)
 
    node = QiUnipatracking()


    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()