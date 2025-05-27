import requests
import os
import json

class WhisperGroq:
    def __init__(self):
        # URL dell'API Groq
        self.API_URL = "https://api.groq.com/openai/v1/audio/transcriptions"
        
        # Token API di Groq
        self.API_TOKEN = os.environ.get("GROQ_API_KEY")
        
        if not self.API_TOKEN:
            raise ValueError("API token non fornito. Specifica il token GROQ_API_KEY nelle variabili d'ambiente.")
        
        # Headers per le richieste API
        self.headers = {
            "Authorization": f"Bearer {self.API_TOKEN}",
            "Accept": "application/json"
        }
    
    def transcribe_audio(self, audio_path):
        try:
            # Preparazione dei file da inviare
            files = {
                'file': (os.path.basename(audio_path), open(audio_path, 'rb'), 'audio/mpeg'),
                'model': (None, 'whisper-large-v3-turbo')  # Specifica il modello Whisper su Groq
            }
            
            # Invio della richiesta
            response = requests.post(
                self.API_URL, 
                headers=self.headers,
                files=files
            )
            
            # Gestione della risposta
            if response.status_code == 200:
                result = response.json()
                return result.get("text", "Trascrizione non disponibile")
            else:
                raise ValueError(f"Errore API Groq: {response.status_code}, {response.text}")
                
        except Exception as e:
            raise RuntimeError(f"Errore durante la trascrizione: {e}")
        finally:
            # Chiusura del file se aperto
            if 'files' in locals() and 'file' in files:
                files['file'][1].close()