import yaml
import time
import os
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ThrowManager:
    """
    負責處理拋投邏輯的獨立類別。
    透過傳入主節點 (main node) 來共用 publisher 與狀態。
    """
    def __init__(self, node):
        # 接收 main.py 的節點實例
        self.node = node

        config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
        
        # --- 姿勢定義： [arm_1 (底座), arm_2 (肩膀), gripper (夾爪)] ---
        # TODO: 請根據實際情況填寫這些數值
        self.pos_ready = self.config['throw']['pos_ready']  # 預備姿勢 (夾爪保持閉合)
        self.pos_accelerate = self.config['throw']['pos_accelerate'] # 加速上揮 (夾爪保持閉合)
        self.pos_release = self.config['throw']['pos_release'] # 出手瞬間 (夾爪全開)
        self.pos_end = self.config['throw']['pos_end']     # 順勢停止 (Follow-through)

    def execute_throw(self):
        self.node.get_logger().info("開始執行拋投序列！")
        
        # 1. 移動到準備姿勢
        self._move_to_ready_position()
        time.sleep(2.0) # 等待手臂穩定
        
        # 2. 發射 (加速上揮與釋放)
        self.node.get_logger().info("發射！")
        msg = JointTrajectory()
        msg.joint_names = self.node.arm_joint_names

        point_accelerate = JointTrajectoryPoint()
        point_accelerate.positions = self.pos_accelerate
        point_accelerate.time_from_start = Duration(sec=0, nanosec=100_000_000) 
        
        point_release = JointTrajectoryPoint()
        point_release.positions = self.pos_release
        point_release.time_from_start = Duration(sec=0, nanosec=150_000_000) 
        
        point_end = JointTrajectoryPoint()
        point_end.positions = self.pos_end
        point_end.time_from_start = Duration(sec=0, nanosec=300_000_000)
        
        msg.points.append(point_accelerate)
        msg.points.append(point_release)
        msg.points.append(point_end)
        
        self.node.arm_publisher_.publish(msg)
        self.node.get_logger().info("拋投完成！")
        
        # 3. 【關鍵】同步狀態回主節點
        # 拋投結束後，更新主節點記錄的手臂位置與夾爪狀態，避免後續遙控暴衝
        self.node.arm_positions = list(self.pos_end)
        self.node.is_gripper_open = True
        
        # 給予一點緩衝時間讓動作做完，再將控制權交還給鍵盤
        time.sleep(1.0) 

    def _move_to_ready_position(self):
        msg = JointTrajectory()
        msg.joint_names = self.node.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.pos_ready
        point.time_from_start = Duration(sec=1, nanosec=500_000_000)
        
        msg.points.append(point)
        self.node.arm_publisher_.publish(msg)
        self.node.get_logger().info("移動至拋投預備位置...")