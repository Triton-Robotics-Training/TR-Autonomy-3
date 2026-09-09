#!/usr/bin/env python3

import sim_node.ros_bridge
import rclpy

def main():
    print("running sim node")
    rclpy.init()
    
    node = None

    try:
        node = sim_node.ros_bridge.Sim_Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("shutting down sim node")
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
