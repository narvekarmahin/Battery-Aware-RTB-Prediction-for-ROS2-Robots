import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import math

class BatteryPredictor(Node):
    def __init__(self):
        super().__init__('battery_predictor')
        
        # Subscriptions
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
            
        # Phase 2: Nav2 Action Client Initialization
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Dock Coordinates
        self.dock_x = 0.0
        self.dock_y = 0.0
        
        # Real-time state
        self.current_battery = 1.0 
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Phase 3: Dynamic Tracking Variables
        self.last_tracked_x = None
        self.last_tracked_y = None
        self.last_tracked_battery = None
        
        self.energy_per_meter = 0.05  # Fallback guess until robot moves
        self.safety_buffer = 0.20     
        self.rtb_triggered = False
        
        self.get_logger().info("Advanced Battery Predictor Node Initialized")

    def battery_callback(self, msg):
        self.current_battery = msg.percentage
        self.update_dynamic_discharge()
        self.check_threshold()

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.update_dynamic_discharge()
        self.check_threshold()

    def update_dynamic_discharge(self):
        # Initialize tracking points if empty
        if self.last_tracked_x is None:
            self.last_tracked_x = self.current_x
            self.last_tracked_y = self.current_y
            self.last_tracked_battery = self.current_battery
            return

        # Calculate distance moved since last track point
        dist_moved = math.sqrt(
            (self.current_x - self.last_tracked_x)**2 + 
            (self.current_y - self.last_tracked_y)**2
        )
        
        # Update the energy rate every 0.5 meters to get stable math
        if dist_moved >= 0.5:
            battery_dropped = self.last_tracked_battery - self.current_battery
            
            if battery_dropped > 0:
                # Calculate new dynamic discharge rate
                self.energy_per_meter = battery_dropped / dist_moved
                self.get_logger().info(f"Recalibrated discharge rate: {self.energy_per_meter*100:.2f}% per meter")
            
            # Reset tracking points for the next 0.5m stretch
            self.last_tracked_x = self.current_x
            self.last_tracked_y = self.current_y
            self.last_tracked_battery = self.current_battery

    def check_threshold(self):
        if self.rtb_triggered:
            return 

        distance_to_dock = math.sqrt((self.dock_x - self.current_x)**2 + (self.dock_y - self.current_y)**2)
        required_battery = (distance_to_dock * self.energy_per_meter) + self.safety_buffer
        
        if self.current_battery <= required_battery:
            self.get_logger().warning(
                f"CRITICAL: Battery at {self.current_battery*100:.1f}%. "
                f"Required to return: {required_battery*100:.1f}%. RTB Initiated!"
            )
            self.execute_rtb()

    def execute_rtb(self):
        self.rtb_triggered = True
        self.get_logger().info("Connecting to Nav2 Action Server...")
        
        # Ensure Nav2 is actually running before sending the command
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 Action Server not available! Cannot return to base.")
            return
        
        # Construct the navigation goal payload
        goal_msg = NavigateToPose()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.dock_x
        goal_msg.pose.pose.position.y = self.dock_y
        goal_msg.pose.pose.orientation.w = 1.0  # Tells the robot to face forward when it parks
        
        self.get_logger().info(f"Sending goal coordinates: X={self.dock_x}, Y={self.dock_y}")
        self.nav_client.send_goal_async(goal_msg)

    def battery_callback(self, msg):
        # Normalize Gazebo's 0-100 scale down to a standard 0.0-1.0 decimal
        self.current_battery = msg.percentage / 100.0 
        
        self.get_logger().info(f"Live Battery: {self.current_battery*100:.2f}%")
        self.update_dynamic_discharge()
        self.check_threshold()

def main(args=None):
    rclpy.init(args=args)
    node = BatteryPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()