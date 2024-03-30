from gtts import gTTS 
from playsound import playsound
import os
import time

def speech(text, language = "en") :
    narrator = gTTS(text=text, lang=language, slow=False)
    file_path = os.getcwd()
    narrator.save(os.path.join(file_path, "narrator_temp.mp3"))
    time.sleep(0.01)
    playsound(os.path.join(file_path, "narrator_temp.mp3"))
    
if __name__ == "__main__" :
    speech("I died and reincarnated into a very thin and sharp pizza crust. and accidentally beheaded the strongest dragon in the world and become the strongest pizza in the world")