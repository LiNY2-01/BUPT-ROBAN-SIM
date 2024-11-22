import math
import rospy
import time
from ik_lib.ikmodulesim import IkModuleSim
from ik_lib.ikmodulesim.CtrlType import CtrlType as C

class ClimbStairs(IkModuleSim):
    def __init__(self):
        super(ClimbStairs, self).__init__()

    def raise_hand(self):
       
        self.body_motion([C.RArm_z, C.RArm_x], [0.165, 0.074], 20) 

        time.sleep(1)
        
        self.body_motion([C.RArm_x, C.RArm_y], [0.073, 0.008], 30)
        
        # self.body_motion([C.RArm_z, C.RArm_x], [0.05, -0.02], 20)  
        
        # self.body_motion([C.RArm_z, C.RArm_x], [-0.05, -0.05], 20)
        # print("over")
        


if __name__ == '__main__':
    rospy.init_node('climb_stairs', anonymous=True)
    climb_stairs = ClimbStairs()

    def shut_down():
        climb_stairs.reset()

    rospy.on_shutdown(shut_down)

    # start the simulation once
    if climb_stairs.toInitPoses():
        climb_stairs.raise_hand()
        climb_stairs.reset()