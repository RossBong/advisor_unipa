import qi
import sys
import rclpy
import time
import os
import wave
import paramiko
from rclpy.node import Node
from std_msgs.msg import Bool,String
from qi_unipa_msgs.msg import PostureWithSpeed,Track


class QiUnipaSpeech(Node):  
    def __init__(self):
        super().__init__('qi_unipa_speech')
        
        # Ottieni i parametri
        self.declare_parameter('ip','192.168.0.161')
        self.declare_parameter('port',9559)
        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        # Connessione sessione
        self.session = self.set_connection(self.ip, port)

        #ALService
        self.memory = self.session.service("ALMemory")
        self.animated_service = self.session.service("ALAnimatedSpeech")
        self.audio_service = self.session.service("ALAudioRecorder")
        self.sound_detect_service = self.session.service("ALSoundDetection")
        self.sound_detect_service.setParameter("Sensitivity", 0.8)
        self.configuration = {"bodyLanguageMode":"contextual"}
        self.leds_service = self.session.service("ALLeds")

        #Variabili
        self.last_index=self.memory.getData("ALTextToSpeech/Status")[0]
        self.is_recognizing = False

        #Topic subscription
        self.tts_sub = self.create_subscription(String, "/speak", self.set_tts, 10)

        self.record_sub = self.create_subscription(Bool, "/record", self.record_callback, 10)

        #Topic publischer
        self.tracking_pub = self.create_publisher(Track, "/track", 10)
        self.posture_pub = self.create_publisher(PostureWithSpeed, "/posture", 10)
        self.isSpeaking_pub=self.create_publisher(Bool,'/is_speaking',10)

        self.stt_pub =self.create_publisher(String,"/stt",10)

      

        
        self.create_timer(0.5, self.check_speaking)
        
    def set_connection(self, ip, port):
        session = qi.Session()
        try:
            session.connect(f"tcp://{ip}:{port}")
        except RuntimeError:
            self.get_logger().error(f"Can't connect to Naoqi at ip \"{ip}\" on port {port}.\n"
                                    "Please check your script arguments.")
            sys.exit(1)
        return session

    def pub_track(self, name, distance):
        msg=Track()
        msg.target_name=name
        msg.distance=distance
        self.tracking_pub.publish(msg)

    def pub_posture(self, name, speed):
        msg=PostureWithSpeed()
        msg.posture_name=name
        msg.speed=speed
        self.posture_pub.publish(msg)

    def set_tts(self, msg):
        self.animated_service.say(msg.data)
        self.pub_posture("Stand", 0.5)

    def set_led(self, on):
        names = [
        "Face/Led/Green/Left/0Deg/Actuator/Value",
        "Face/Led/Green/Left/45Deg/Actuator/Value",
        "Face/Led/Green/Left/90Deg/Actuator/Value",
        "Face/Led/Green/Left/135Deg/Actuator/Value",
        "Face/Led/Green/Left/180Deg/Actuator/Value",
        "Face/Led/Green/Left/225Deg/Actuator/Value",
        "Face/Led/Green/Left/270Deg/Actuator/Value",
        "Face/Led/Green/Left/315Deg/Actuator/Value",

        "Face/Led/Green/Right/0Deg/Actuator/Value",
        "Face/Led/Green/Right/45Deg/Actuator/Value",
        "Face/Led/Green/Right/90Deg/Actuator/Value",
        "Face/Led/Green/Right/135Deg/Actuator/Value",
        "Face/Led/Green/Right/180Deg/Actuator/Value",
        "Face/Led/Green/Right/225Deg/Actuator/Value",
        "Face/Led/Green/Right/270Deg/Actuator/Value",
        "Face/Led/Green/Right/315Deg/Actuator/Value"]

        self.leds_service.createGroup("eyes",names)
        # Switch the new group on
        if on==True:
            self.leds_service.off("FaceLeds")
            self.leds_service.on("eyes")
        elif on==False:
            self.leds_service.off("eyes")
            self.leds_service.on("FaceLeds")
            
            
    def record_callback(self, msg):
       
        # Configurazione della registrazione
        channels = [1, 1, 1, 1]  # Abilitare tutti e 4 i microfoni (frontale, posteriore, sinistro, destro)
        audio_format = "wav"
        sample_rate = 16000  # Frequenza di campionamento (16 kHz)
        output_file_robot = "/home/nao/audio_record_unipa/recording.wav"

        # Avviare la registrazione
        
        self.sound_detect_service.subscribe("Audio Detection")
        self.audio_service.startMicrophonesRecording(output_file_robot, audio_format, sample_rate, channels)
        self.get_logger().info("(Qi Unipa Speech: Avvio microfoni")
        self.set_led(True)
        time.sleep(0.3)
        while not self.is_recognizing:
            time.sleep(0.3)
            
            if  self.memory.getData("SoundDetected")[0][1]==1:
                self.get_logger().info("(Qi Unipa Speech: Avvio registrazione...")
                
                self.is_recognizing=True
                
        # Attendere la fine della registrazione
        while self.is_recognizing:
                time.sleep(2)
                if  self.memory.getData("SoundDetected")[0][1]==0:
                    self.is_recognizing=False
                     # Terminare la registrazione
                    self.audio_service.stopMicrophonesRecording()
                    self.set_led(False)
                    self.sound_detect_service.unsubscribe("Audio Detection")
                    self.get_logger().info(f"Registrazione terminata e salvata in: {output_file_robot}")
        

        path_ros_ws=os.path.join(os.path.abspath(__file__).split("/install")[0])
     
        # Trasferire il file al PC
        local_output_file = os.path.join(path_ros_ws,"src/audio/recording.wav")
        
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(self.ip, username='nao', password='nao')#'192.168.0.161'

        sftp = ssh.open_sftp()
        sftp.get(output_file_robot, local_output_file)
        sftp.close()
        ssh.close()
        self.get_logger().info("File trasferito con successo!")
        
        res=String()

        res.data=local_output_file
        self.stt_pub.publish(res)
        
      
    def check_speaking(self):
        msg=Bool()
        status=self.memory.getData("ALTextToSpeech/Status")
               
        if status[1]=="done" and status[0]!=self.last_index:
            msg.data=False
            self.isSpeaking_pub.publish(msg)
        else :
            msg.data=True
            self.isSpeaking_pub.publish(msg)

        self.last_index=status[0]

def main(args=None):
    rclpy.init(args=args)
 
    node = QiUnipaSpeech()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()