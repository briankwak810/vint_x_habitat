#!/usr/bin/env python3

import rospy
from ros_x_habitat.srv import ResetAgent, GetAgentTime
from std_msgs.msg import String
import time

# Import from ros_x_habitat
import sys
sys.path.append('/home/kjsbrian/projects/vint_x_habitat/src/ros_x_habitat/src')
from constants.constants import AgentResetCommands, PACKAGE_NAME, ServiceNames

class VintAgentServices:
    """
    Minimal agent service provider for ViNT integration.
    Provides only the services required by HabitatROSEvaluator.
    """
    
    def __init__(self, node_name="vint_agent_node"):
        self.node_name = node_name
        rospy.init_node(self.node_name)
        
        # Timing tracking for agent performance metrics
        self.episode_start_time = None
        self.total_agent_time = 0.0
        self.action_count = 0
        
        # Setup service servers
        self.reset_service = rospy.Service(
            f"{PACKAGE_NAME}/{self.node_name}/{ServiceNames.RESET_AGENT}",
            ResetAgent,
            self.reset_agent
        )
        
        self.get_agent_time_service = rospy.Service(
            f"{PACKAGE_NAME}/{self.node_name}/{ServiceNames.GET_AGENT_TIME}",
            GetAgentTime,
            self.get_agent_time
        )
        
        # Publishers for ViNT communication
        self.reset_pub = rospy.Publisher('/vint/reset', String, queue_size=1)
        self.status_pub = rospy.Publisher('/vint/status', String, queue_size=1)
        
        # Subscribe to ViNT action timing
        self.vint_action_sub = rospy.Subscriber('/vint/action_time', String, self.action_time_callback)
        
        rospy.loginfo(f"ViNT agent services node '{self.node_name}' started")
    
    def reset_agent(self, request):
        """
        Handle agent reset requests from evaluator.
        Maps to ViNT reset signal.
        """
        if request.reset == AgentResetCommands.RESET:
            rospy.loginfo(f"Resetting ViNT agent with seed: {request.seed}")
            
            # Reset timing metrics
            self.episode_start_time = time.time()
            self.total_agent_time = 0.0
            self.action_count = 0
            
            # Signal ViNT to reset
            reset_msg = String()
            reset_msg.data = f"reset,seed={request.seed}"
            self.reset_pub.publish(reset_msg)
            
            # Brief delay to ensure reset is processed
            rospy.sleep(0.1)
            
            return True
            
        elif request.reset == AgentResetCommands.SHUTDOWN:
            rospy.loginfo("Shutting down ViNT agent services")
            
            # Signal ViNT to shutdown
            shutdown_msg = String()
            shutdown_msg.data = "shutdown"
            self.status_pub.publish(shutdown_msg)
            
            return True
    
    def get_agent_time(self, request):
        """
        Return average agent time per action.
        For ViNT, this represents navigation computation time.
        """
        if self.action_count == 0:
            return 0.0
        
        avg_time = self.total_agent_time / self.action_count
        rospy.logdebug(f"Agent time: {avg_time:.4f}s (total: {self.total_agent_time:.4f}s, actions: {self.action_count})")
        return avg_time
    
    def action_time_callback(self, msg):
        """
        Track ViNT action computation time.
        Expected format: "action_time=0.123"
        """
        try:
            time_str = msg.data.split('=')[1]
            action_time = float(time_str)
            self.total_agent_time += action_time
            self.action_count += 1
        except (IndexError, ValueError) as e:
            rospy.logwarn(f"Invalid action time message: {msg.data}, error: {e}")
    
    def spin(self):
        """Keep the service node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        service_node = VintAgentServices()
        service_node.spin()
    except rospy.ROSInterruptException:
        pass
