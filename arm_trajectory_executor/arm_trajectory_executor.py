import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

import time

class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__("arm_trajectory_executor")
		
        self.traj_sub = self.create_subscription(JointTrajectory, '/arm/joint_trajectory', self.trajectory_callback, 10)
        
        self.state_pub = self.create_publisher(JointState, '/arm/joint_targets', 10)

        self.executing = False

        self.get_logger().info("Trajectory executor ready")

    def trajectory_callback(self, traj: JointTrajectory):
        if self.executing:
            self.get_logger().warn("already executing trajectory, ignoring new one")
            return

        if not traj.points:
            self.get_logger().warn("received empty trajectory")
            return

        self.executing = True
        self.get_logger().info(f"Executing traj with {len(traj.points)} points")

        for point in traj.points:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = traj.joint_names
            msg.position = list(point.positions)

            self.state_pub.publish(msg)
            time.sleep(0.025)

        self.executing = False
        self.get_logger().info("Trajectory execution complete")

def main():
	rclpy.init()
	node = TrajectoryExecutor()
	rclpy.spin(node)
	rclpy.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
