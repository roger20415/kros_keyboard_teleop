import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys
import termios
import tty
import select

# 終端機的操作提示訊息
msg = """
小車與機械臂遙控節點已啟動！(ROS 2 Jazzy)
---------------------------------------
【底盤控制】
一般移動:
        w
   a    s    d
快跑模式 (按住 Shift + WASD):
        W
   A    S    D

【手臂控制】 (每次按下移動 0.1 弧度)
   i : 手臂關節 2 (Shoulder) 增加
   k : 手臂關節 2 (Shoulder) 減少
   j : 手臂關節 1 (Base) 增加
   l : 手臂關節 1 (Base) 減少
   u : 夾爪 (Gripper) 打開/減少
   o : 夾爪 (Gripper) 閉合/增加

q : 結束程式
空白鍵 / 其他按鍵 : 底盤煞車停止
---------------------------------------
"""

# 底盤按鍵對應 (線速度係數 x, 角速度係數 z)
moveBindings = {
    'w': (1.0, 0.0),
    'a': (0.0, 1.0),
    's': (-1.0, 0.0),
    'd': (0.0, -1.0),
}

# 手臂按鍵對應 (關節索引, 增減弧度)
# index 0: arm_1_joint, index 1: arm_2_joint, index 2: gripper_joint
armBindings = {
    'l': (0, -0.1),
    'j': (0, 0.1),
    'k': (1, -0.1),
    'i': (1, 0.1),
    'u': (2, -0.1),
    'o': (2, 0.1),
}

class CustomTeleopNode(Node):
    """
    Node class responsible for handling keyboard input and publishing 
    both Twist and JointTrajectory messages.
    """
    def __init__(self):
        """
        Initialize the node, publishers, speed parameters, and arm limits.
        """
        super().__init__('custom_teleop_node')
        
        # 底盤 Publisher
        self.cmd_publisher_ = self.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)
        
        # 手臂 Publisher (使用 Topic 發送軌跡)
        self.arm_publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # 底盤速度設定
        self.normal_speed = 0.2
        self.normal_turn = 1.0
        self.sprint_speed = 0.546
        self.sprint_turn = 3.983

        # 手臂初始目標角度 (Base, Shoulder, Gripper)
        # 範圍限制為 0.0 ~ 4.189 rad (對應硬體 0~240度)
        self.arm_positions = [2.1, 2.1, 2.1]
        self.arm_joint_names = ['arm_1_joint', 'arm_2_joint', 'gripper_joint']

    def get_key(self, settings, timeout=0.1):
        """
        Read a single key press from the terminal using raw mode.
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def publish_arm_command(self):
        """
        Constructs and publishes a JointTrajectory message based on current arm_positions.
        """
        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.arm_positions
        # 設定手臂到達目標的時間 (200 毫秒，使其動作平滑且反應快速)
        point.time_from_start = Duration(sec=0, nanosec=200_000_000)
        
        msg.points.append(point)
        self.arm_publisher_.publish(msg)

    def run(self):
        """
        Main loop to process keyboard events and publish commands.
        """
        settings = termios.tcgetattr(sys.stdin)
        print(msg)

        x = 0.0
        th = 0.0
        status = 0
        
        current_speed = self.normal_speed
        current_turn = self.normal_turn

        try:
            while rclpy.ok():
                key = self.get_key(settings)

                # --- 處理手臂控制 ---
                if key.lower() in armBindings.keys():
                    joint_idx, step = armBindings[key.lower()]
                    self.arm_positions[joint_idx] += step
                    
                    # 限制角度在安全範圍內 (0.0 ~ 4.189 rad)
                    self.arm_positions[joint_idx] = max(0.0, min(4.189, self.arm_positions[joint_idx]))
                    
                    # 只有按手臂按鍵時才發送手臂指令
                    self.publish_arm_command()
                    continue # 處理完手臂就不處理底盤邏輯

                # --- 處理底盤控制 ---
                elif key.lower() in moveBindings.keys():
                    if key.isupper():
                        current_speed = self.sprint_speed
                        current_turn = self.sprint_turn
                    else:
                        current_speed = self.normal_speed
                        current_turn = self.normal_turn
                        
                    x = moveBindings[key.lower()][0]
                    th = moveBindings[key.lower()][1]

                elif key == 'q' or key == '\x03': 
                    break # 結束程式
                    
                else:
                    x = 0.0
                    th = 0.0
                    if key == '' and status == 0:
                        pass # 如果沒按按鍵，維持上一次的速度(或者為0)

                # 發佈底盤 TwistStamped 訊息
                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.twist.linear.x = x * current_speed
                twist_msg.twist.angular.z = th * current_turn
                self.cmd_publisher_.publish(twist_msg)
                
        except Exception as e:
            self.get_logger().error(f"執行時發生錯誤: {e}")
            
        finally:
            # 發送停止指令確保小車底盤停止
            empty_twist = TwistStamped()
            empty_twist.header.stamp = self.get_clock().now().to_msg()
            # twist 預設為零向量，直接發佈即可
            self.cmd_publisher_.publish(empty_twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    """
    Entry point for the custom teleop node.
    """
    rclpy.init(args=args)
    node = CustomTeleopNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()