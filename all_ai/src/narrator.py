from gtts import gTTS 
from playsound import playsound
import os

def speech(text, language = "en") :
    narrator = gTTS(text=text, lang=language, slow=False)
    file_path = os.getcwd()
    narrator.save(os.path.join(file_path, "narrator_temp.mp3"))
    playsound(os.path.join(file_path, "narrator_temp.mp3"))
    
if __name__ == "__main__" :
    print("Start")
    speech("I died and reincarnated into a very thin and sharp pizza crust. and accidentally beheaded the strongest dragon in the world and become the strongest pizza in the world")
    print("Done")