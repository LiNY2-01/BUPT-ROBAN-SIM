## ik_lib

- **ikmodulesim.py**

  class IkModuleSim(object):

  > 封装机器人逆解相关操作，主要使用 toInitPose ， body_motion 和 reset 方法
  
  ```python
  def toInitPoses(self):
      #机器人位姿初始化，在动作执行前调用
      #初始化成功 返回 True ,否则，返回 False
  ```

  ```python
  def body_motion(self, body_pose, value, count=100):
      # 机器人末端动作执行，给定目标位姿 body_pose，value 通过逆解得到舵机角度，控制机器人运动
      Args:
          - body_pose: CtrlType, or [CtrlType] 
              	 表示末端位置，如左/右脚(Lfoot/Rfoot)，身体中心(Torso)
          - value: list, or two dim list. RPY, xyz.
              	 表示给相应末端的位姿值，[r, p, y, x, y, z]
          - count: int. times of division
                   插值次数，缺省为 100
  ```

  ```python
  def reset(self):
      # 动作执行完后的复位，在动作完成后调用
  ```

  使用方法：需从ik_lib中导入 IkModuleSim 和 CtrlType， 然后新建一个类，继承自 IkModuleSim, 然后调用上述方法，完成自定义动作，示例如下：

  ```python
  import math
  import rospy
  from ik_lib.ikmodulesim import IkModuleSim
  from ik_lib.ikmodulesim.CtrlType import CtrlType as C
  
  class ClimbStairs(IkModuleSim):
      def __init__(self):
          super(ClimbStairs, self).__init__()
  
      def climb_stairs(self):
          # 重心下降 4cm
          self.body_motion(C.Torso, [0, 0, 0, 0, 0.0, -0.04])  
          # 重心向左腿偏移 12cm
          self.body_motion(C.Torso_y, 0.12)  
          # 右腿向上移动 5cm
          self.body_motion(C.Rfoot, [0, 0, 0, 0, 0.0, 0.05])  
          # 右腿向前20cm
          self.body_motion([C.Torso, C.Rfoot],
                           [[0, -math.pi / 8, 0, 0.0, 0, 0], 
                            [0, 0, 0, 0.20, 0.0, 0.0]]) 
          # 右腿下2cm
          self.body_motion([C.Torso, C.Rfoot],
                           [[0, math.pi / 8, 0, 0.08, -0.12, 0.0], 
                            [0, 0, 0, 0, 0.0, -0.02]])  
          # 抬左腿1
          self.body_motion([C.Torso, C.Lfoot], 
                           [[math.pi / 8, math.pi / 6, 0, 0.16, -0.10, 0.05],
                            [0, -math.pi / 6, 0, 0.06, 0.0, 0.14]])  
          # 左腿向前20cm
          self.body_motion([C.Torso, C.Lfoot], 
                           [[0.0, -math.pi / 9, 0, -0.04, 0.0, 0.0],
                            [0, math.pi / 6, 0, 0.14, 0.0, -0.05]])  
          # 复原
          self.body_motion([C.Torso, C.Lfoot],
                           [[-math.pi / 8, -math.pi / 18, 0, 0, 0.10, 0.02], 
                            [0, 0, 0, 0.0, 0.0, -0.06]]) 
  
  
  if __name__ == '__main__':
      rospy.init_node('climb_stairs', anonymous=True)
      climb_stairs = ClimbStairs()
  
      def shut_down():
          climb_stairs.reset()
  
      rospy.on_shutdown(shut_down)
  
      # start the simulation once
      if climb_stairs.toInitPoses():
          climb_stairs.climb_stairs()
          climb_stairs.reset()
  ```

  

  

  ## 