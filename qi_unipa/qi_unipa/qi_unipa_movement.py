import qi
import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
from qi_unipa_msgs.msg import PostureWithSpeed, JointAnglesWithSpeed, Hand

class QiUnipa_Movement(Node):
    def __init__(self):
        super().__init__('qi_unipa_movement')

        # Ottieni i parametri
        self.declare_parameter('ip','192.168.0.161')
        self.declare_parameter('port',9559)
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        # Connessione sessione
        self.session = self.set_connection(ip, port)

        self.motion_service = self.session.service("ALMotion")
        self.posture_service = self.session.service("ALRobotPosture")

        self.state_sub = self.create_subscription(Int32, "/state", self.set_state, 10)
        self.angles_sub= self.create_subscription(JointAnglesWithSpeed, "/set_angles", self.set_angles, 10)
        self.walk_sub = self.create_subscription(Vector3, "/walk", self.set_walking, 10)
        self.posture_sub = self.create_subscription(PostureWithSpeed, "/posture", self.set_posture, 10)
        self.hands_sub = self.create_subscription(Hand, "/hands", self.set_hand, 10)
        self.position_pub = self.create_publisher(Vector3, "/position", 10)

        self.timer = self.create_timer(1.0, self.get_position)


    def set_connection(self, ip, port):
        session = qi.Session()
        try:
            session.connect(f"tcp://{ip}:{port}")
        except RuntimeError:
            self.get_logger().error(f"Can't connect to Naoqi at ip \"{ip}\" on port {port}.\n"
                                    "Please check your script arguments.")
            sys.exit(1)
        return session

    def set_state(self, msg):
        state = msg.data
        if state == 0:
            self.motion_service.wakeUp()
        else:
            self.motion_service.rest()

    def set_angles(self, msg):
        names = msg.names
        angles = msg.angles.tolist()
        speed = msg.speed
        self.motion_service.setAngles(names, angles, speed)

    def set_walking(self, msg):
        x = msg.x
        y = msg.y
        theta = msg.z
        self.motion_service.moveToward(x,y,theta)

    def set_posture(self, msg):
        pose_name = msg.posture_name
        speed = msg.speed
        self.posture_service.goToPosture(pose_name, speed)
    
    def set_hand(self, msg):
        hand = msg.hand
        fun = msg.fun
        if hand == "Hands":
            if fun == 0:
                self.motion_service.openHand("RHand")
                self.motion_service.openHand("LHand")
            else:
                self.motion_service.closeHand("RHand")
                self.motion_service.closeHand("LHand")
        else:
            if fun == 0:
                self.motion_service.openHand(hand)
            else:
                self.motion_service.closeHand(hand)

    def get_position(self):
        pose=self.motion_service.getRobotPosition(False)
        msg=Vector3()
        msg.x=pose[0]
        msg.y=pose[1]
        msg.y=pose[2]
        self.position_pub.publish(msg)
    
    
def main(args=None):
    rclpy.init(args=args)
    
    node = QiUnipa_Movement()
    
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()