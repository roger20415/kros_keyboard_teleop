import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import sys
import termios
import tty
import select
import yaml
import os

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
空白鍵 : 夾爪一鍵開/關，且底盤煞車停止
其他按鍵 : 底盤煞車停止
---------------------------------------
"""

# (key: (linear, angular))
moveBindings = {
    'w': (1.0, 0.0),
    'a': (0.0, 1.0),
    's': (-1.0, 0.0),
    'd': (0.0, -1.0),
}

# index 0: arm_1_joint, index 1: arm_2_joint, index 2: gripper_joint
# index 1: rotate direction
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
    both Twist and JointTrajectory messages, while monitoring joint states.
    """
    def __init__(self):
        """
        Initialize the node, load configurations, publishers, subscribers, and set up variables.
        """
        super().__init__('custom_teleop_node')
        
        config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)

        self.normal_speed = self.config['base']['normal_speed']
        self.normal_turn = self.config['base']['normal_turn']
        self.sprint_speed = self.config['base']['sprint_speed']
        self.sprint_turn = self.config['base']['sprint_turn']
        
        self.arm_max = self.config['arm']['max_rad']
        self.arm_min = self.config['arm']['min_rad']
        self.gripper_max = self.config['gripper']['max_rad']
        self.gripper_min = self.config['gripper']['min_rad']
        
        self.base_publisher_ = self.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)
        self.arm_publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.arm_positions = [
            (self.arm_max + self.arm_min) / 2, 
            (self.arm_max + self.arm_min) / 2, 
            (self.gripper_max + self.gripper_min) / 2
        ]
        self.arm_joint_names = ['arm_1_joint', 'arm_2_joint', 'gripper_joint']
        
        self.initialized_from_state = False
        
        self.is_gripper_open = True

    def joint_state_callback(self, msg):
        """
        Callback to update the initial arm positions from the actual joint states.
        """
        if not self.initialized_from_state:
            try:
                idx_1 = msg.name.index('arm_1_joint')
                idx_2 = msg.name.index('arm_2_joint')
                idx_g = msg.name.index('gripper_joint')
                
                self.arm_positions[0] = msg.position[idx_1]
                self.arm_positions[1] = msg.position[idx_2]
                self.arm_positions[2] = msg.position[idx_g]
                
                gripper_threshold = (self.gripper_max + self.gripper_min) / 2.0
                if self.arm_positions[2] > gripper_threshold:
                    self.is_gripper_open = True
                else:
                    self.is_gripper_open = False
                
                self.initialized_from_state = True
                
                state_str = "開啟" if self.is_gripper_open else "閉合"
                self.get_logger().info(f"已成功讀取手臂與夾爪當前角度。夾爪目前狀態: {state_str}。可以開始遙控。")

            except ValueError:
                pass

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
        point.time_from_start = Duration(sec=0, nanosec=200_000_000)
        
        msg.points.append(point)
        self.arm_publisher_.publish(msg)

    def run(self):
        """
        Main loop to process keyboard events, spin for callbacks, and publish commands.
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
                rclpy.spin_once(self, timeout_sec=0.0)
                
                key = self.get_key(settings)

                if key.lower() in armBindings.keys() and not self.initialized_from_state:
                    self.get_logger().warn("尚未讀取到夾爪與手臂角度，請稍候再試...")
                    continue

                if key.lower() in armBindings.keys():
                    joint_idx, step = armBindings[key.lower()]
                    self.arm_positions[joint_idx] += step
                    
                    if joint_idx == 2:
                        self.arm_positions[joint_idx] = max(self.gripper_min, min(self.gripper_max, self.arm_positions[joint_idx]))
                    else:
                        self.arm_positions[joint_idx] = max(self.arm_min, min(self.arm_max, self.arm_positions[joint_idx]))
                    
                    self.publish_arm_command()
                    continue 
                
                elif key == ' ':
                    if not self.initialized_from_state:
                        self.get_logger().warn("尚未讀取到夾爪角度，請稍候再試...")
                        continue
                        
                    gripper_threshold = (self.gripper_max + self.gripper_min) / 2.0
                    self.is_gripper_open = self.arm_positions[2] > gripper_threshold
                    
                    self.is_gripper_open = not self.is_gripper_open
                    
                    if self.is_gripper_open:
                        self.arm_positions[2] = self.gripper_max
                        self.get_logger().info("夾爪狀態：全開")
                    else:
                        self.arm_positions[2] = self.gripper_min
                        self.get_logger().info("夾爪狀態：全關")
                        
                    self.publish_arm_command()
                    
                    x = 0.0
                    th = 0.0

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
                    break 
                    
                else:
                    x = 0.0
                    th = 0.0
                    if key == '' and status == 0:
                        pass

                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.twist.linear.x = x * current_speed
                twist_msg.twist.angular.z = th * current_turn
                self.base_publisher_.publish(twist_msg)
                
        except Exception as e:
            self.get_logger().error(f"執行時發生錯誤: {e}")
            
        finally:
            empty_twist = TwistStamped()
            empty_twist.header.stamp = self.get_clock().now().to_msg()
            self.base_publisher_.publish(empty_twist)
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