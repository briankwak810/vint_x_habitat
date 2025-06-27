#!/usr/bin/env python3

from src.nodes.habitat_env_node import HabitatEnvNode, main
import argparse

def vint_main():
    """ViNT-specific launcher for habitat_env_node with proper defaults"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--node-name", type=str, default="vint_env_node")
    parser.add_argument("--task-config", type=str, default="configs/pointnav_d_orignal.yaml")
    parser.add_argument("--enable-physics-sim", default=True, action="store_true")  # Default True for ViNT
    parser.add_argument("--use-continuous-agent", default=True, action="store_true")  # Default True for ViNT
    parser.add_argument("--sensor-pub-rate", type=float, default=20.0)
    args = parser.parse_args()

    # Initialize with ViNT-specific parameters
    env_node = HabitatEnvNode(
        node_name=args.node_name,
        config_paths=args.task_config,
        enable_physics_sim=args.enable_physics_sim,
        use_continuous_agent=args.use_continuous_agent,  # Always True for ViNT
        pub_rate=args.sensor_pub_rate,
    )

    # Override topic subscription for ViNT
    import rospy
    from geometry_msgs.msg import Twist
    
    # Replace the subscriber with ViNT-specific topic
    env_node.sub.unregister()  # Remove original subscriber
    env_node.sub = rospy.Subscriber(
        "/mobile_base/commands/velocity", Twist, env_node.callback, queue_size=env_node.sub_queue_size
    )
    
    # Run simulations
    env_node.simulate()

if __name__ == "__main__":
    vint_main()