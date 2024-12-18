import qi
import sys
import rclpy
import re
import time
import os
import wave
import paramiko
from rclpy.node import Node
from std_msgs.msg import Bool,String,Int32,ByteMultiArray
from qi_unipa_msgs.msg import PostureWithSpeed,Track
from qi_unipa_msgs.srv import Speak

class QiUnipaSpeech(Node):  
    def __init__(self):
        super().__init__('qi_unipa_speech')
        
        # Ottieni i parametri
        self.declare_parameter('ip', '192.168.0.161')
        self.declare_parameter('port', 9559)
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        # Connessione sessione
        self.session = self.set_connection(ip, port)

        self.is_recognizing = False
        self.last_word=""
        self.vocabulary = ["ciao pepper", "come stai", "stop parlato"]
        
        self.reply={"ciao pepper":"Ciao ^start(animations/Stand/BodyTalk/BodyTalk_1) io sono Pepper, piacere di conoscerti ^wait(animations/Stand/BodyTalk/BodyTalk_1) ",
                    "come stai": " Io sto bene e tu come ti senti?",
                    "stop parlato":"Ciao Ciao",
                   }
        
        
        self.asr_service = self.session.service("ALSpeechRecognition")
        self.memory = self.session.service("ALMemory")
        self.animated_service = self.session.service("ALAnimatedSpeech")
        self.audio_service = self.session.service("ALAudioRecorder")
        self.sound_detect_service = self.session.service("ALSoundDetection")
        self.sound_detect_service.setParameter("Sensitivity", 0.8)
        self.configuration = {"bodyLanguageMode":"contextual"}
        self.leds_service = self.session.service("ALLeds")
        
        self.speech_sub = self.create_subscription(Bool, "/listen", self.set_speech, 10)
        self.tts_sub = self.create_subscription(String, "/speak", self.set_tts, 10)

        #self.record_sub = self.create_subscription(Int32, "/record", self.record, 10)
        self.tracking_pub = self.create_publisher(Track, "/track", 10)
        self.audio_publisher = self.create_publisher(ByteMultiArray,'/audio', 10)
        self.posture_pub = self.create_publisher(PostureWithSpeed, "/posture", 10)
        self.record_srv=self.create_service(Speak,'record',self.record)
        # Setup iniziale del riconoscimento vocale
        self.setup_recognition()
        
        # Timer per il controllo periodico del riconoscimento vocale
        self.create_timer(1.0, self.check_recognition)
        self.isSpeaking_pub=self.create_publisher(Bool,'/is_speaking',10)
        self.create_timer(0.5, self.check_speaking)
        self.last_index=self.memory.getData("ALTextToSpeech/Status")[0]

    def set_connection(self, ip, port):
        session = qi.Session()
        try:
            session.connect(f"tcp://{ip}:{port}")
        except RuntimeError:
            self.get_logger().error(f"Can't connect to Naoqi at ip \"{ip}\" on port {port}.\n"
                                    "Please check your script arguments.")
            sys.exit(1)
        return session

    def setup_recognition(self):
        try:
            # Imposta la lingua in italiano
            self.asr_service.setLanguage("Italian")
            self.get_logger().info("Lingua impostata a Italiano")
            
            # Imposta il vocabolario
            #self.asr_service.setVocabulary(self.vocabulary, True)
            self.get_logger().info(f"Vocabolario impostato: {self.vocabulary}")
            
            # Creazione del proxy per l'evento
            self.memory.declareEvent("WordRecognized")
            self.get_logger().info("Evento WordRecognized dichiarato")

        except Exception as e:
            self.get_logger().info(f"Errore nella configurazione: {e}")
            raise

    def set_speech(self, msg):
        if msg.data and not self.is_recognizing:
            self.start_recognition()

        elif not msg.data and self.is_recognizing:
            self.stop_recognition()
            self.pub_track("Stop", 3.0)

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
  
    def start_recognition(self):
        try:
            self.pub_track("Sound", 1.0)
            self.asr_service.subscribe("SpeechRecognition")
            self.is_recognizing = True
            self.get_logger().info("Riconoscimento vocale avviato")
        except Exception as e:
            self.get_logger().info(f"Errore nell'avvio del riconoscimento: {e}")
            self.is_recognizing = False
        
    def stop_recognition(self):
        try:
            #self.pub_posture("Stand", 0.5)
            self.asr_service.unsubscribe("SpeechRecognition")
            self.is_recognizing = False
            self.get_logger().info("Riconoscimento vocale fermato")
        except Exception as e:
             self.get_logger().info(f"Errore nell'arresto del riconoscimento vocale: {e}")

    def answers(self,word):
        if word  in self.reply:
            self.get_logger().info(f'risposta :{self.reply[word]}')
            ans=self.reply[word]
            self.stop_recognition()
            self.animated_service.say(ans)
            self.pub_posture("Stand", 0.5)
            self.start_recognition()
            if word =="stop parlato" and self.is_recognizing :
                self.stop_recognition()
                self.pub_track("Stop", 3.0)
            
    def check_recognition(self):
        if not self.is_recognizing:
            return
            
        try:
            word_data = self.memory.getData("WordRecognized")
            
            self.memory.insertData("WordRecognized","") #evita di leggere ripetutamente la stessa parola
            if word_data:
                self.get_logger().info(f"Sentence: {word_data}")
                word =re.sub(r'\W+', ' ', word_data[0]).strip() # Regex per eliminare tutto ciò che non è una parola
                
            if word_data and word_data!=self.last_word:
                confidence = word_data[1]
                self.get_logger().info(f"Parola riconosciuta: {word} (confidenza: {confidence})")
                if confidence > 0.455 :
                    #self.get_logger().info(f"Parola riconosciuta: {word} (confidenza: {confidence})")
                    self.answers(word)
                    self.last_word=word
                    
        except Exception as e:
             self.get_logger().info(f"Errore durante il controllo del riconoscimento: {e}")

    def set_tts(self, msg):
        self.animated_service.say(msg.data)
        self.pub_posture("Stand", 0.5)

         
    def led_eyes(self, on):
        names = [
        "Face/Led/Red/Left/0Deg/Actuator/Value",
        "Face/Led/Red/Left/90Deg/Actuator/Value",
        "Face/Led/Red/Left/180Deg/Actuator/Value",
        "Face/Led/Red/Left/270Deg/Actuator/Value"]

        self.leds_service.createGroup("eyes",names)
        # Switch the new group on
        if on==True:
            self.leds_service.on("FaceLeds")
        elif on==False:
            self.leds_service.off("FaceLeds")
            
    def record(self,request,response):
       
        # Configurazione della registrazione
        channels = [1, 1, 1, 1]  # Abilitare tutti e 4 i microfoni (frontale, posteriore, sinistro, destro)
        audio_format = "wav"
        sample_rate = 16000  # Frequenza di campionamento (16 kHz)
        duration=1 # Durata della registrazione in secondi
        output_file_robot = "/home/nao/audio_record_unipa/test_pepper1.wav"

        # Avviare la registrazione
        
        self.sound_detect_service.subscribe("Audio Detection")
        self.audio_service.startMicrophonesRecording(output_file_robot, audio_format, sample_rate, channels)
        self.get_logger().info("(Qi Unipa Speech:Avvio microfoni")
        self.led_eyes(True)
        time.sleep(0.3)
        while not self.is_recognizing:
            time.sleep(0.3)
            
            if  self.memory.getData("SoundDetected")[0][1]==1:
                self.get_logger().info("(Qi Unipa Speech:Avvio registrazione...")
                
                self.is_recognizing=True
                
        # Attendere la fine della registrazione
        while self.is_recognizing:
                time.sleep(2)
                if  self.memory.getData("SoundDetected")[0][1]==0:
                    self.is_recognizing=False
                     # Terminare la registrazione
                    self.audio_service.stopMicrophonesRecording()
                    self.led_eyes(False)
                    self.sound_detect_service.unsubscribe("Audio Detection")
                    self.get_logger().info(f"Registrazione terminata e salvata in: {output_file_robot}")

        
        path_ros_ws=os.path.join(os.path.abspath(__file__).split("/install")[0])
     
        # Trasferire il file al PC
        local_output_file = os.path.join(path_ros_ws,"src/audio/audio_recording.wav")
        
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect('192.168.254.173', username='nao', password='nao')#'192.168.0.161'

        sftp = ssh.open_sftp()
        sftp.get(output_file_robot, local_output_file)
        sftp.close()
        ssh.close()

        
        self.get_logger().info("File trasferito con successo!")
        
        response.resp=True
        return response
   


    def publish_audio(self, file_path):
        try:
            # Leggi il file WAV
            with wave.open(file_path, 'rb') as wav_file:
                frames = wav_file.readframes(wav_file.getnframes())
                audio_msg = ByteMultiArray(data=frames)
                self.audio_publisher.publish(audio_msg)
                self.get_logger().info(f"File audio pubblicato sul topic 'audio_topic'.")
        except Exception as e:
            self.get_logger().error(f"Errore durante la pubblicazione del file audio: {e}")

    def check_speaking(self):
        msg=Bool()
        status=self.memory.getData("ALTextToSpeech/Status")
               
        if status[1]=="done" and status[0]!=self.last_index:
            
            msg.data=False
            self.isSpeaking_pub.publish(msg)
        else :
            
            self.get_logger().info(f"last_index:{self.last_index}")
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