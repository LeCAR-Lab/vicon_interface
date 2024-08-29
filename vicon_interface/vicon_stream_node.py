from threading import Thread
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from pyvicon_datastream import tools
import tf_transformations

import numpy as np

from vicon_interface.vicon_data_buffer import ViconDataBuffer, ViconDataFilter


class ViconStreamNode(Node):
    def __init__(self):
        super().__init__("vicon_stream_node")
        self.viacon_tracker_ip_ = (
            self.declare_parameter("vicon_tracker_ip", rclpy.Parameter.Type.STRING)
            .get_parameter_value()
            .string_value
        )
        self.viacon_object_name_ = (
            self.declare_parameter("vicon_object_name", rclpy.Parameter.Type.STRING)
            .get_parameter_value()
            .string_value
        )
        self.min_states_ = np.array(
            self.declare_parameter("min_states", rclpy.Parameter.Type.DOUBLE_ARRAY)
            .get_parameter_value()
            .double_array_value
        )
        self.max_states_ = np.array(
            self.declare_parameter("max_states", rclpy.Parameter.Type.DOUBLE_ARRAY)
            .get_parameter_value()
            .double_array_value
        )
        self.parent_frame_id_ = (
            self.declare_parameter("odom_frame_id", "map")
            .get_parameter_value()
            .string_value
        )
        self.child_frame_id_ = (
            self.declare_parameter("odom_child_frame_id", "base_link")
            .get_parameter_value()
            .string_value
        )
        self.poll_vicon_dt_ = (
            self.declare_parameter("poll_vicon_dt", 0.01)
            .get_parameter_value()
            .double_value
        )
        self.filtered_odom_dt_ = (
            self.declare_parameter("filtered_odom_dt", 0.02)
            .get_parameter_value()
            .double_value
        )
        self.rolling_window_size_ = (
            self.declare_parameter("rolling_window_size", 10)
            .get_parameter_value()
            .integer_value
        )

        self.buffer_ = ViconDataBuffer(6, self.rolling_window_size_)
        # self.buffer_ = ViconDataFilter(1.0, 0.01)

        self.get_logger().info(
            f"Connecting to Viacon tracker at {self.viacon_tracker_ip_}..."
        )
        self.tracker_ = tools.ObjectTracker(self.viacon_tracker_ip_)
        if self.tracker_.is_connected:
            self.get_logger().info(
                f"Connection to {self.viacon_tracker_ip_} successful"
            )
        else:
            self.get_logger().fatal(f"Connection to {self.viacon_tracker_ip_} failed")
            raise Exception(f"Connection to {self.viacon_tracker_ip_} failed")

        self.pose_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "vicon_pose", 1
        )
        self.odometry_publisher_ = self.create_publisher(Odometry, "odometry/vicon", 1)
        self.filtered_odometry_publisher_ = self.create_publisher(
            Odometry, "odometry/vicon/filtered", 1
        )
        self.latency_publisher_ = self.create_publisher(Float32, "vicon_latency", 1)
        self.tf_broadcaster_ = TransformBroadcaster(self)

        self.cb_group_poll_viacon_ = MutuallyExclusiveCallbackGroup()
        self.cb_group_filtered_odom_ = MutuallyExclusiveCallbackGroup()

        # self.timer_ = self.create_timer(self.poll_vicon_dt_, self.timer_callback, self.cb_group_poll_viacon_)
        self.last_frame_num_ = 0
        self.poll_vicon_thread = Thread(target=self.timer_callback)
        self.poll_vicon_thread.start()
        
        self.filtered_odom_timer_ = self.create_timer(
            self.filtered_odom_dt_, self.filtered_odom_timer_callback, self.cb_group_filtered_odom_
        )

    def timer_callback(self):
        subsample_counter = 1
        while True:
            # import time
            begin = time.monotonic()
            print("tracking", self.viacon_object_name_)
            position = self.tracker_.get_position(self.viacon_object_name_)
            end_1 = time.monotonic()
            # if subsample_counter % 4 == 0:
            #     subsample_counter = 1
            # else:
            #     subsample_counter += 1
            #     continue
            # end = time.time()
            if not position:
                self.get_logger().warn(
                    f"Cannot get the pose of `{self.viacon_object_name_}`.",
                    throttle_duration_sec=1.0,
                )
                return

            latency = position[0]
            frame_num = position[1]
            if frame_num == self.last_frame_num_:
                return
            self.last_frame_num_ = frame_num
            print("position", position)
            # if len(position[2]) == 0:
            #     print("no data")
            #     continue
            try:
                obj = position[2][0]
                _, _, x, y, z, roll, pitch, yaw = obj
                q = tf_transformations.quaternion_from_euler(roll, pitch, yaw, 'rxyz')
                roll, pitch, yaw = tf_transformations.euler_from_quaternion(q, 'sxyz')
                self.buffer_.add_pose((x, y, z, roll, pitch, yaw), latency)
            except Exception as e:
                print(e)
                self.get_logger().warn(f"Vicon dropped a frame", throttle_duration_sec=1.0)
                
            print("here")

            v_body, ang_v_body, _ = self.buffer_.get_latest_velocity()
            if _ == 0:
                return
            position, _ = self.buffer_.get_latest_pose()
            x, y, z, roll, pitch, yaw = position

            states = np.array(
                [x, y, z, roll, pitch, yaw, v_body[0], v_body[1], v_body[2], ang_v_body[0], ang_v_body[1], ang_v_body[2]]
            )
            states = np.clip(states, self.min_states_, self.max_states_)
            x, y, z, roll, pitch, yaw, v_body[0], v_body[1], v_body[2], ang_v_body[0], ang_v_body[1], ang_v_body[2] = states

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.parent_frame_id_
            odom_msg.child_frame_id = self.child_frame_id_
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = z
            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            odom_msg.twist.twist.linear.x = v_body[0]
            odom_msg.twist.twist.linear.y = v_body[1]
            odom_msg.twist.twist.linear.z = v_body[2]
            odom_msg.twist.twist.angular.x = ang_v_body[0]
            odom_msg.twist.twist.angular.y = ang_v_body[1]
            odom_msg.twist.twist.angular.z = ang_v_body[2]
            self.odometry_publisher_.publish(odom_msg)

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.pose.pose.position.x = x
            pose_msg.pose.pose.position.y = y
            pose_msg.pose.pose.position.z = z
            pose_msg.pose.pose.orientation.x = q[0]
            pose_msg.pose.pose.orientation.y = q[1]
            pose_msg.pose.pose.orientation.z = q[2]
            pose_msg.pose.pose.orientation.w = q[3]
            pose_msg.header.stamp = odom_msg.header.stamp
            pose_msg.header.frame_id = self.parent_frame_id_
            self.pose_publisher_.publish(pose_msg)

            transform = TransformStamped()
            transform.header.stamp = odom_msg.header.stamp
            transform.header.frame_id = self.parent_frame_id_
            transform.child_frame_id = self.child_frame_id_
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]
            # self.tf_broadcaster_.sendTransform(transform)

            latency_msg = Float32()
            latency_msg.data = latency
            self.latency_publisher_.publish(latency_msg)
            end = time.monotonic()
            if end - begin < self.poll_vicon_dt_:
                time.sleep(self.poll_vicon_dt_ - (end - begin))

    def filtered_odom_timer_callback(self):
        if not self.buffer_.is_ready():
            return
        position, orientation, time_stamp = self.buffer_.get_interpolated_pose()
        v_body, ang_v_body, _ = self.buffer_.get_latest_velocity()

        x, y, z = position
        roll, pitch, yaw = orientation
        states = np.array(
            [x, y, z, roll, pitch, yaw, v_body[0], v_body[1], v_body[2], ang_v_body[0], ang_v_body[1], ang_v_body[2]]
        )
        states = np.clip(states, self.min_states_, self.max_states_)
        x, y, z, roll, pitch, yaw, v_body[0], v_body[1], v_body[2], ang_v_body[0], ang_v_body[1], ang_v_body[2] = states

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.parent_frame_id_
        odom_msg.child_frame_id = self.child_frame_id_
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = v_body[0]
        odom_msg.twist.twist.linear.y = v_body[1]
        odom_msg.twist.twist.linear.z = v_body[2]
        odom_msg.twist.twist.angular.x = ang_v_body[0]
        odom_msg.twist.twist.angular.y = ang_v_body[1]
        odom_msg.twist.twist.angular.z = ang_v_body[2]
        self.filtered_odometry_publisher_.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    vicon_stream_node = ViconStreamNode()
    executor = MultiThreadedExecutor()
    executor.add_node(vicon_stream_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    # vicon_stream_node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
