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

        # self.current_trajectory = None
        # self.traj_start_time = None
        # self.last_point_idx = 0

        # self.create_timer(0.02, self.trajectory_step)

        self.get_logger().info("Trajectory executor ready")

    def trajectory_callback(self, traj: JointTrajectory):
        if self.executing:
            self.get_logger().warn("already executing trajectory, ignoring new one")
            return

        if not traj.points:
            self.get_logger().warn("received empty trajectory")
            return

        # self.current_trajectory = traj
        # self.traj_start_time = self.get_clock().now()
        # self.last_point_idx = 0

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

    # def trajectory_step(self):
    #     if not self.executing or self.current_trajectory is None:
    #         return

    #     traj = self.current_trajectory
    #     elapsed = (self.get_clock().now() - self.traj_start_time).nanoseconds / 1e9

    #     point_idx = 0
    #     for i, point in enumerate(traj.points):
    #         point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
    #         if elapsed >= point_time:
    #             point_idx = i
    #         else:
    #             break

    #     if point_idx >= len(traj.points) - 1:
    #         final_point = traj.points[-1]
    #         msg = JointState()
    #         msg.header.stamp = self.get_clock().now().to_msg()
    #         msg.name = traj.joint_names
    #         msg.position = list(final_point.positions)
    #         self.state_pub.publish(msg)

    #         self.executing = False
    #         self.current_trajectory = None
    #         self.get_logger().info("trajectory execution complete")
    #         return

    #     if point_idx < len(traj.points) - 1:
    #         p1 = traj.points[point_idx]
    #         p2 = traj.points[point_idx + 1]
    #         
    #         t1 = p1.time_from_start.sec + p1.time_from_start.nanosec / 1e9
    #         t2 = p2.time_from_start.sec + p2.time_from_start.nanosec / 1e9

    #         if t2 > t1:
    #             alpha = (elapsed - t1) / (t2 - t1)
    #             alpha = max(0.0, min (1.0, alpha))

    #             interpolated_pos = [p1.positions[i] + alpha * (p2.positions[i] - p1.positions[i]) for i in range(len(p1.positions))]

    #             msg = JointState()
    #             msg.header.stamp = self.get_clock().now().to_msg()
    #             msg.name = traj.joint_names
    #             msg.position = interpolated_pos
    #             self.state_pub.publish(msg)

def main():
	rclpy.init()
	node = TrajectoryExecutor()
	rclpy.spin(node)
	rclpy.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
