import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

import time

class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__("arm_trajectory_executor")
		
        self.traj_sub = self.create_subscription(JointTrajectory, '/arm/joint_trajectory', self.trajectory_callback, 10)

        self.emergency_sub = self.create_subscription(Bool, '/arm/emergency_stop', self.emergency_callback, 10)
        
        self.exec_sub = self.create_subscription(String, '/arm/execution_state', self.exec_callback, 10)

        self.state_pub = self.create_publisher(JointState, '/arm/joint_targets', 10)

        self.exec_state = "IDLE"
        self.current_trajectory = True

        self.emergency_active = False

        self.get_logger().info("Trajectory executor ready")

    def exec_callback(self, msg: String):
        self.exec_state = msg.data

    def emergency_callback(self, msg: Bool):
        if msg.data:
            if not self.emergency_active:
                self.get_logger().error("EMERGENCY STOP ACTIVATED")
            self.emergency_active = True

        else:
            if self.emergency_active:
                self.get_logger().info("Emergency stop cleared")
            self.emergency_active = False

    def trajectory_callback(self, traj: JointTrajectory):
        if self.emergency_active:
            self.get_logger().warn("Blocking traj due to emergency")
            return

        if self.current_trajectory is not None:
            self.get_logger().warn("Executor busy, ignoring new trajectory")
            return

        if self.exec_state in ["PLANNING", "EXECUTING"]:
            self.get_logger().warn(f"Ignoring trajectory; IK state = {self.exec_state}")
            return

        if not traj.points:
            self.get_logger().warn("received empty trajectory")
            return

        self.current_trajectory = traj
        self.get_logger().info(f"Executing traj with {len(traj.points)} points")

        for point in traj.points:
            if self.emergency_active:
                self.get_logger().error("execution interrupted by emergency stop")
                break

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = traj.joint_names
            msg.position = list(point.positions)

            self.state_pub.publish(msg)
            time.sleep(0.025)

        self.current_trajectory = None

        if not self.emergency_active:
            self.get_logger().info("Trajectory execution complete")
        else:
            self.get_logger().warn("Trajectory terminated due to emergency")

def main():
	rclpy.init()
	node = TrajectoryExecutor()
	rclpy.spin(node)
	rclpy.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
