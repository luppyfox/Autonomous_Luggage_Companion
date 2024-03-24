import tinyik
import numpy as np
import pyFileInteraction as fi

class simple_ik() :
    def __init__(self, titf = None) -> None :
        self.titf = titf
        if self.titf != None :
            self.arm = tinyik.Actuator(self.titf)
        else :
            pass
        
    def load_titf(self, json_path) :
        data = fi.read_json(json_path)
        titf_name = data["name"]
        unit = data["unit"]
        self.titf = data["titf"]
        self.arm = tinyik.Actuator(self.titf)
        return titf_name, unit
    
    def create_titf(self, titf) :
            self.arm = tinyik.Actuator(titf)
        
    def ik(self, target : list) :
        if self.titf == None :
            e = "TITF is not declare. Please call 'load_titf' or 'create_titf' before using IK"
            raise Exception(e)
        self.arm.ee = target
        return list(np.rad2deg(self.arm.angles))
    
    
if __name__ == "__main__" :
    ik = simple_ik()
    ik.load_titf("titf_json/manipulator_titf2.json")
    
    target = [200, 300, 200]
    print(ik.ik(target))