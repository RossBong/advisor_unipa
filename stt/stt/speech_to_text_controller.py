import rclpy
import wave
import time
from rclpy.node import Node
from std_msgs.msg import Bool,String,Int32


from .whisper_groq import WhisperGroq



class Speech_Controller(Node):
    def __init__(self):
        super().__init__("speech_controller")

        self.stt_sub = self.create_subscription(String, "/stt", self.stt_callback, 10)

        self.transcription_pub =self.create_publisher(String,"/transcription",10)

        self.whisper = WhisperGroq()
  

             

    def stt_callback(self,msg):
            self.get_logger().info(f"path: {msg.data}")
            transcription=self.whisper.transcribe_audio(msg.data)
            self.get_logger().info(f"trascrizione effettuata {transcription}")

            res=String()
            res.data=transcription
            self.transcription_pub.publish(res)
             


def main(args=None):
    rclpy.init(args=args)
 
    node = Speech_Controller()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
