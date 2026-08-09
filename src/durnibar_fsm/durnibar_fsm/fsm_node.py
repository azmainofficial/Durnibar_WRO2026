import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from enum import Enum

class RobotState(Enum):
    BOOT_WAIT = 1       # Power On (Waiting for Start Button - Rule 9.11)
    START_LEAVE = 2     # Exit Start Section & Init Lap Counter
    LANE_FOLLOW = 3     # Normal driving & corner handling
    OBSTACLE_EVADE = 4  # Traffic sign side-passing (Red=Right, Green=Left)
    LAP3_COMPLETE = 5   # 3 Laps completed
    PARK_SEARCH = 6     # Search for 20cm parking lot gap using LiDAR
    PARALLEL_PARK = 7   # 2-Phase reverse parallel parking maneuver
    AUTONOMOUS_STOP = 8 # Full stop

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')

        self.current_state = RobotState.BOOT_WAIT
        self.lap_count = 0

        # ROS 2 Subscribers & Publishers
        self.sign_sub = self.create_subscription(String, '/traffic_sign', self.sign_callback, 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)

        # Timer for state machine loop (20 Hz)
        self.timer = self.create_timer(0.05, self.fsm_loop)

        self.get_logger().info('WRO 2026 High-Level FSM Node Initialized!')

    def sign_callback(self, msg: String):
        if self.current_state == RobotState.LANE_FOLLOW:
            if msg.data != "NONE":
                self.current_state = RobotState.OBSTACLE_EVADE
                self.get_logger().info(f'Traffic Sign Detected: {msg.data}')

    def fsm_loop(self):
        state_msg = String()
        state_msg.data = self.current_state.name
        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
