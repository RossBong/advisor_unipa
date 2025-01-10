import rclpy
import wave
import time
from rclpy.node import Node
from std_msgs.msg import Bool,String,Int32

from qi_unipa_msgs.srv import Record, Stt
from .llm_dieta4v import Explainability



class Explainability_Controller(Node):
    def __init__(self):
        super().__init__("explainability_controller")
        
        self.transcription_sub = self.create_subscription(String, "/transcription", self.transcription_callback, 10)

        self.record_pub =self.create_publisher(Bool,"/record",10)

        self.isSpeaking_sub = self.create_subscription(Bool, "/is_speaking",self.check_speaking,10)
        self.speak_pub=self.create_publisher(String,"/speak",10)
        self.exp=Explainability()


    def transcription_callback(self, msg):
        text=msg.data.lower()
        self.get_logger().info(f" {text}")

        if ('termina' not in text):
           
            res=String()
            res.data=self.exp.chat(text)
            self.speak_pub.publish(res)

        else:
            res=String()
            res.data='È stato un piacere aiutarti. A presto!'
            self.speak_pub.publish(res)
            self.destroy_node()
            rclpy.shutdown()
            exit()
    

          
    
    def check_speaking(self,msg):
       msg2=Bool()
       msg2.data=True
       if not msg.data:
         self.record_pub.publish(msg2) #ascolta
    
    def start_conversation(self):
        msg=String()
        msg.data="Ciao sono Pepper ! Come posso aiutarti oggi ?"
        self.speak_pub.publish(msg)
    
    
        




        


def main(args=None):
    rclpy.init(args=args)
 
    node = Explainability_Controller()
    node.start_conversation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    
    
    main()
   