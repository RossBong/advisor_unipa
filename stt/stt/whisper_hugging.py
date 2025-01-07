import requests
import os
class WhisperHugging:
    def __init__(self):
        """
        Inizializza il client per l'API Whisper di Hugging Face.
        
        :param api_url: URL dell'API (di default per il modello Whisper).
        :param api_token: Token di autenticazione per Hugging Face.
        """
        self.API_URL = "https://api-inference.huggingface.co/models/openai/whisper-large-v3-turbo"
        self.API_TOKEN = os.environ.get("WHISPER_API_KEY")

        # Inserisci qui il token di default se necessario.
        
        if not self.API_TOKEN:
            raise ValueError("API token non fornito. Specifica il token nella classe o tramite parametro.")
        
        self.headers = {"Authorization": f"Bearer {self.API_TOKEN}"}

    def transcribe_audio(self, audio_path):
        """
        Trascrive un file audio usando l'API Hugging Face.
        
        :param audio_path: Percorso al file audio.
        :return: La trascrizione come stringa.
        """
        try:
            with open(audio_path, "rb") as audio_file:
                audio_bytes = audio_file.read()
            
            response = requests.post(self.API_URL, headers=self.headers, data=audio_bytes)
            
            if response.status_code == 200:
                result = response.json()
                return result.get("text", "Trascrizione non disponibile")
            else:
                raise ValueError(f"Errore API: {response.status_code}, {response.text}")
        except Exception as e:
            raise RuntimeError(f"Errore durante la trascrizione: {e}")




