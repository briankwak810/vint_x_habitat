import shlex
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
from std_msgs.msg import String

# Import from ros_x_habitat
from src.evaluators.habitat_ros_evaluator import HabitatROSEvaluator
from ros_x_habitat.srv import EvalEpisode, ResetAgent, GetAgentTime
from src.constants.constants import NumericalMetrics
from src.evaluators.habitat_sim_evaluator import HabitatSimEvaluator
from src.constants.constants import (
    AgentResetCommands,
    EvalEpisodeSpecialIDs,
    PACKAGE_NAME,
    ServiceNames,
)
from src.utils import utils_logging

class VintHabitatEvaluator(HabitatROSEvaluator):
    def __init__(self, *args, **kwargs):
        # Override do_not_start_nodes to handle agent node manually
        kwargs['do_not_start_nodes'] = True
        # Set agent node name for ViNT services
        kwargs['agent_node_name'] = 'vint_agent_node'
        super().__init__(*args, **kwargs)

        self.start_nodes()
    
    def start_nodes(self):
        """Start complete ViNT-Habitat system"""
        # 1. Start ViNT agent services first
        agent_services_args = shlex.split(
            f"python /home/kjsbrian/projects/vint_ws/src/vint_habitat/vint_agent_services.py"
        )
        self.agent_process = subprocess.Popen(agent_services_args)
        rospy.loginfo(f"Started ViNT agent services with PID: {self.agent_process.pid}")
        
        # 2. Start ViNT-Habitat bridge
        bridge_args = shlex.split(
            f"python /home/kjsbrian/projects/vint_ws/src/vint_habitat/vhab_bridge.py"
        )
        self.bridge_process = subprocess.Popen(bridge_args)
        rospy.loginfo(f"Started ViNT-Habitat bridge with PID: {self.bridge_process.pid}")
        
        # # 3. Start ViNT navigation launcher
        # nav_launcher_args = shlex.split(
        #     f"python /home/kjsbrian/projects/vint_ws/src/vint_habitat/vint_nav_launcher.py"
        # )
        # self.nav_launcher_process = subprocess.Popen(nav_launcher_args)
        # rospy.loginfo(f"Started ViNT navigation launcher with PID: {self.nav_launcher_process.pid}")
        
        # 4. Start environment node (using ViNT launcher for proper topic mapping)
        env_script = "/home/kjsbrian/projects/vint_ws/src/vint_habitat/vint_env_launcher.py"
        
        base_args = [
            "python", env_script,
            "--node-name", self.env_node_name,
            "--task-config", self.config_paths,
            "--sensor-pub-rate", str(self.sensor_pub_rate)
        ]
        
        # Physics and continuous agent are defaults in ViNT launcher
        if not self.enable_physics:
            base_args.append("--no-physics-sim")  # Override default if needed
            
        self.env_process = subprocess.Popen(base_args)
        rospy.loginfo(f"Started ViNT Habitat env node with PID: {self.env_process.pid}")
        
        
        # Brief delay to ensure all services are available
        rospy.sleep(2.0)
        rospy.loginfo("Complete ViNT-Habitat system started")
    
    def shutdown_agent_node(self):
        """Override to handle complete ViNT system shutdown"""
        processes = [
            ("ViNT agent services", getattr(self, 'agent_process', None)),
            ("ViNT-Habitat bridge", getattr(self, 'bridge_process', None)),
            ("ViNT navigation launcher", getattr(self, 'nav_launcher_process', None))
        ]
        
        for name, process in processes:
            if process is not None:
                rospy.loginfo(f"Shutting down {name}")
                process.terminate()
                process.wait()
        
        rospy.loginfo("All ViNT processes shut down")