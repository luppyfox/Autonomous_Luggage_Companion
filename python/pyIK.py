from ikpy.chain import Chain
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class urdf_simple_ik() :
    def __init__(self, ufrd_path : str) -> None :
        self.arm = Chain.from_urdf_file(ufrd_path)
    
    def ik(self, target : list) -> list :
        armrad = self.arm.inverse_kinematics(
            target_position=target
            )
        return list(armrad)
    
    def debug_plot(self, armrad : list) -> None :
        ax = plt.figure().add_subplot(111, projection='3d')
        self.arm.plot(armrad, ax)
        plt.show()
    
    
if __name__ == "__main__" :
    ik_simple = urdf_simple_ik("manipulator_tf2/urdf/manipulator_tf2.urdf")
    armrad = ik_simple.ik(target = [0.1, 0.2, 0.3])
    print(armrad)
    ik_simple.debug_plot(armrad)