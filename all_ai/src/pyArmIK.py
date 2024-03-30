import tinyik
import numpy as np
import pyFileInteraction as fi
from serial import Serial
import time
from struct import pack

class armIk() :
    def __init__(self, connect_port, baud = 9600, titf_json_path = None) -> None :
        self.ser = Serial(port = connect_port, baudrate=baud, timeout=1, writeTimeout=1)
        
        self.titf = None
        if titf_json_path != None :
            self.load_titf(titf_json_path)
        else :
            pass
        
    def move_arm(self, cartesian_position) :
        mani_dag = [0, 0, 0, 0]
        arm_deg = self.ik(cartesian_position)
        mani_dag[0] = arm_deg[0] + 90
        mani_dag[1] = arm_deg[1] + 90
        mani_dag[2] = arm_deg[2] + 90
        mani_dag[3] = - arm_deg[3]
        if min(mani_dag) < 0 or max(mani_dag) > 180 :
            print("Cannot go to position!")
            # mani_dag = [90, 90, 90, 90]
        mani_dag[0] = 0 if mani_dag[0] < 0 else mani_dag[0]
        mani_dag[1] = 0 if mani_dag[1] < 0 else mani_dag[1]
        mani_dag[2] = 0 if mani_dag[2] < 0 else mani_dag[2]
        mani_dag[3] = 0 if mani_dag[3] < 0 else mani_dag[3]
        mani_dag[0] = 180 if mani_dag[0] > 180 else mani_dag[0]
        mani_dag[1] = 180 if mani_dag[1] > 180 else mani_dag[1]
        mani_dag[2] = 180 if mani_dag[2] > 180 else mani_dag[2]
        mani_dag[3] = 180 if mani_dag[3] > 180 else mani_dag[3]
        if mani_dag[1] + mani_dag[2] < 90 :
            mani_dag[1] = 0
            mani_dag[2] = 90
        self.send_command(mani_dag)
        
    def send_command(self, data_list) :
        # print(data_list)
        str_data_list = []
        for dat in data_list :
            str_data_list.append(int(dat))
        s = pack('hhhh', *str_data_list)
        # print(s)
        self.ser.write(s)
        
    def load_titf(self, titf_json_path) :
        data = fi.read_json(titf_json_path)
        titf_name = data["name"]
        unit = data["unit"]
        self.titf = data["titf"]
        self.arm = tinyik.Actuator(self.titf, optimizer=tinyik.ScipySmoothOptimizer())
        return titf_name, unit
        
    def ik(self, target : list) :
        if self.titf == None :
            e = "TITF is not declare. Please call 'load_titf' or 'create_titf' before using IK"
            raise Exception(e)
        self.arm.ee = target
        return list(np.rad2deg(self.arm.angles))  
    
if __name__ == "__main__" :
    # import keyboard as ky
    ik = armIk(connect_port="COM3", baud=9600  , titf_json_path="/home/luppyfox/catkin_ws/src/Autonomous_Luggage_Companion/all_ai/src/titf_json/manipulator_titf2.json")
    target_1 = [90, 180, 45, 180]
    target_2 = [90, 180, 10, 180]
    target_3 = [90, 90, 60, 140]
    target_4 = [90, 90, 60, 130]
    target_5 = target_1
    target_list = [target_1, target_2, target_3, target_4, target_5]
    # time.sleep(5)
    for tar in target_list :
        # time.sleep(5)
        print(tar)
        for i in range(100) :
            ik.send_command(tar)
            time.sleep(0.02)
        # time.sleep(3)