#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Bool
import yaml
import numpy as np
import time

class VintHabitatBridge:
    def __init__(self):
        rospy.init_node('vint_habitat_bridge')
        
        # Load configurations
        self.load_configs()
        
        # Subscribers - listen to ViNT outputs
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.waypoint_sub = rospy.Subscriber('/waypoint', PointStamped, self.waypoint_callback)
        self.vint_status_sub = rospy.Subscriber('/vint/status', String, self.vint_status_callback)
        self.vint_reset_sub = rospy.Subscriber('/vint/reset', String, self.vint_reset_callback)
        
        # Publishers - send to Habitat and agent services
        self.habitat_cmd_pub = rospy.Publisher('/habitat/cmd_vel', Twist, queue_size=1)
        self.action_time_pub = rospy.Publisher('/vint/action_time', String, queue_size=1)
        
        # State management
        self.mode = "teleop"  # "teleop" or "navigate"
        self.navigation_active = False
        self.current_waypoint = None
        self.episode_active = False
        
        # Control parameters
        self.max_linear_vel = 0.4
        self.max_angular_vel = 0.8
        
        # PD Controller parameters for waypoint navigation
        self.kp_linear = 1.0
        self.kp_angular = 2.0
        self.max_waypoint_distance = 2.0
        
        # Performance tracking
        self.action_start_time = None
        
        # Timer for navigation updates
        self.nav_timer = rospy.Timer(rospy.Duration(0.1), self.navigation_step)
        
        rospy.loginfo("ViNT-Habitat Bridge initialized")
    
    def load_configs(self):
        """Load ViNT configuration files"""
        try:
            # Load robot config
            robot_config_path = "/home/kjsbrian/projects/vint_ws/src/visualnav-transformer/deployment/config/robot.yaml"
            with open(robot_config_path, 'r') as f:
                self.robot_config = yaml.safe_load(f)
            
            # Load joystick config
            joy_config_path = "/home/kjsbrian/projects/vint_ws/src/visualnav-transformer/deployment/config/joystick.yaml"
            with open(joy_config_path, 'r') as f:
                self.joy_config = yaml.safe_load(f)
                
            rospy.loginfo("Loaded ViNT configurations")
        except Exception as e:
            rospy.logwarn(f"Could not load ViNT configs: {e}")
            # Use default values
            self.joy_config = {
                'deadman_switch': 0,
                'lin_vel_button': 1,
                'ang_vel_button': 0
            }
    
    def joy_callback(self, msg):
        """Handle joystick input - same logic as ViNT's joy_teleop.py"""
        if self.mode != "teleop":
            return
            
        # Get button/axis indices from config
        deadman_switch = self.joy_config.get('deadman_switch', 0)
        lin_vel_axis = self.joy_config.get('lin_vel_button', 1)
        ang_vel_axis = self.joy_config.get('ang_vel_button', 0)
        
        twist = Twist()
        
        # Check deadman switch
        if len(msg.buttons) > deadman_switch and msg.buttons[deadman_switch]:
            # Apply velocity commands
            if len(msg.axes) > lin_vel_axis:
                twist.linear.x = self.max_linear_vel * msg.axes[lin_vel_axis]
            if len(msg.axes) > ang_vel_axis:
                twist.angular.z = self.max_angular_vel * msg.axes[ang_vel_axis]
                
            rospy.logdebug(f"Teleop command: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")
        
        # Always publish (zeros when deadman not pressed)
        self.habitat_cmd_pub.publish(twist)
    
    def waypoint_callback(self, msg):
        """Handle ViNT navigation waypoints"""
        if self.mode == "navigate":
            self.start_action_timing()  # Start timing when waypoint received
            self.current_waypoint = np.array([msg.point.x, msg.point.y])
            rospy.loginfo(f"Received waypoint: [{msg.point.x:.2f}, {msg.point.y:.2f}]")
    
    def navigation_step(self, event):
        """Execute one navigation step using PD controller"""
        if self.mode != "navigate" or self.current_waypoint is None:
            return
            
        # Convert waypoint to velocity command using PD controller
        twist = self.waypoint_to_twist(self.current_waypoint)
        self.habitat_cmd_pub.publish(twist)
        
        # Clear waypoint after processing
        self.current_waypoint = None
    
    def waypoint_to_twist(self, waypoint):
        """Convert ViNT waypoint to Twist command using PD controller"""
        twist = Twist()
        
        # Calculate distance and angle to waypoint
        distance = np.linalg.norm(waypoint)
        angle = np.arctan2(waypoint[1], waypoint[0])
        
        # Clamp distance for safety
        distance = min(distance, self.max_waypoint_distance)
        
        # PD controller
        linear_vel = self.kp_linear * distance
        angular_vel = self.kp_angular * angle
        
        # Apply velocity limits
        linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        rospy.logdebug(f"Waypoint to twist: dist={distance:.2f}, angle={angle:.2f}, lin={linear_vel:.2f}, ang={angular_vel:.2f}")
        
        return twist
    
    def vint_status_callback(self, msg):
        """Handle status messages from ViNT or Habitat"""
        if msg.data == "navigation_start":
            self.mode = "navigate"
            self.navigation_active = True
            rospy.loginfo("Switched to navigation mode")
        elif msg.data == "navigation_stop":
            self.mode = "teleop"
            self.navigation_active = False
            rospy.loginfo("Switched to teleop mode")
        elif msg.data == "reset_complete":
            rospy.loginfo("Environment reset completed")
    
    def vint_reset_callback(self, msg):
        """Handle reset signals from evaluator"""
        rospy.loginfo(f"ViNT reset signal received: {msg.data}")
        
        if "reset" in msg.data:
            # Reset ViNT state
            self.navigation_active = False
            self.current_waypoint = None
            self.mode = "navigate"  # Switch to navigation mode for evaluation
            self.episode_active = True
            
            # Extract seed if provided
            if "seed=" in msg.data:
                seed = msg.data.split("seed=")[1]
                rospy.loginfo(f"Episode starting with seed: {seed}")
            
        elif "shutdown" in msg.data:
            rospy.loginfo("Shutdown signal received")
            self.episode_active = False
            rospy.signal_shutdown("Evaluation complete")
    
    def start_action_timing(self):
        """Start timing for action computation"""
        self.action_start_time = time.time()
    
    def end_action_timing(self):
        """End timing and publish action computation time"""
        if self.action_start_time is not None:
            action_time = time.time() - self.action_start_time
            time_msg = String()
            time_msg.data = f"action_time={action_time:.6f}"
            self.action_time_pub.publish(time_msg)
            self.action_start_time = None

    def set_mode(self, mode):
        """Manually set operation mode"""
        if mode in ["teleop", "navigate"]:
            self.mode = mode
            rospy.loginfo(f"Mode set to: {mode}")
        else:
            rospy.logwarn(f"Invalid mode: {mode}")

if __name__ == '__main__':
    try:
        bridge = VintHabitatBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass