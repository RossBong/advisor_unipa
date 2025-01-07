import rclpy
import wave
import time
from rclpy.node import Node
from std_msgs.msg import Bool,String,Int32


from .whisper_hugging import WhisperHugging
from qi_unipa_msgs.srv import Speak


class Speech_Controller(Node):
    def __init__(self):
        super().__init__("speech_controller")

        self.robot_speech_sub = self.create_subscription(String, "/robot_speech",self.robot_speech,10) # topic 1

        self.user_transcription_pub = self.create_publisher(String, "/transcription",10)    #topic 2
        #self.non_lo_so = self.create_service(Speak, )
        self.isSpeaking_sub = self.create_subscription(Bool, "/is_speaking",self.check_speaking,10) # Ci vuole

        self.record_cli=self.create_client(Speak,'record') # ci vuole

        while not self.record_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req=Speak.Request()

        self.whisper = WhisperHugging()
        self.is_speaking=False
        self.isDone=False
  
    def service_callback(self,future):#ritorno del service record
             #self.get_logger().info(f"future: {future.result()}")
             if future.done():
                try:
                    transcription=self.STT(future.result().path)#trascrizione 
                    msg2=String()
                    msg2.data=transcription
                    self.user_transcription_pub.publish(msg2)
                    
                    self.get_logger().info(f"trascrizione inviata {transcription}")

                    #return response.resp  #completamento audio
                except Exception as e:
                    self.get_logger().error(f"Chiamata al servizio fallita: {e}")
                    #return None
             

    def robot_speech(self,msg):
             self.get_logger().info("robot speech in ascolto..")
             
             time.sleep(3)

             self.req.data=10
             future=self.record_cli.call_async(self.req)
             future.add_done_callback(self.service_callback)

             response=self.service_callback(future)


    def STT(self,path):
        #Speech to Text by whisper-large-v3-turbo"
        try:
            
            # Trascrivi un file audio
            transcription = self.whisper.transcribe_audio(path)
            self.get_logger().info(f"risposta ricevuta da whisper :{transcription}")
            return transcription
        except Exception as e:
            self.get_logger().error(f"Errore durate richiesta trascrizione whisper {e}")

    
    def check_speaking(self,msg):
        self.is_speaking=msg.data
         
         


def main(args=None):
    rclpy.init(args=args)
 
    node = Speech_Controller()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
