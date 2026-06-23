#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # Required for TF2 pose transformations
import time
import threading

class PbvsSignWizard(Node):
    def __init__(self):
        super().__init__('pbvs_sign_wizard')
        
        # Adjust this if your EE frame is different
        self.ee_frame = 'Rizon4s-063387_flange'
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers & Subscribers
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/vision/target_pose', self.pose_callback, 10)
        
        self.current_ee_pose = None
        self.test_velocity = 0.2  # 10 cm/s (Very safe/slow)
        self.test_duration = 2.5   # 1.2 seconds
        
        self.get_logger().info("PBVS Sign Wizard Started. Waiting for ArUco marker...")

    def pose_callback(self, msg: PoseStamped):
        # Strip timestamp to get the latest available transform
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        
        try:
            # Transform the camera pose into the robot's End-Effector frame
            transformed_pose = self.tf_buffer.transform(msg, self.ee_frame, rclpy.duration.Duration(seconds=0.1))
            self.current_ee_pose = transformed_pose.pose.position
        except Exception as e:
            pass # Suppress TF warnings to keep terminal clean during testing

    def publish_twist(self, x=0.0, y=0.0, z=0.0):
        msg = TwistStamped()
        msg.header.frame_id = self.ee_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.linear.z = z
        self.twist_pub.publish(msg)

    def test_axis(self, axis_name, axis_index):
        print(f"\n--- TESTING {axis_name.upper()} AXIS ---")
        input(f"Press [ENTER] to nudge the robot slightly in +{axis_name.upper()}...")
        
        if self.current_ee_pose is None:
            print("ERROR: No marker detected! Please ensure the ArUco marker is in view.")
            return 1
            
        # 1. Record starting position
        start_val = [self.current_ee_pose.x, self.current_ee_pose.y, self.current_ee_pose.z][axis_index]
        
        # 2. Command movement
        v = [0.0, 0.0, 0.0]
        v[axis_index] = self.test_velocity
        
        t_end = time.time() + self.test_duration
        while time.time() < t_end:
            self.publish_twist(x=v[0], y=v[1], z=v[2])
            time.sleep(0.01)
            
        # 3. Stop movement and wait for settling
        self.publish_twist(0.0, 0.0, 0.0)
        time.sleep(0.5)
        
        # 4. Record ending position
        end_val = [self.current_ee_pose.x, self.current_ee_pose.y, self.current_ee_pose.z][axis_index]
        
        # 5. Calculate Delta and Deduction
        delta = end_val - start_val
        print(f"[{axis_name.upper()}] Marker moved by: {delta:.4f} meters")
        
        # If commanding +V makes the marker's coordinate decrease (it gets closer), the frame is standard.
        if delta < 0:
            print(f"Result: Standard Alignment. sign_{axis_name} = 1")
            return 1
        else:
            print(f"Result: Reversed Alignment. sign_{axis_name} = -1")
            return -1

    def run_wizard(self):
        # Wait for the first pose to arrive
        while self.current_ee_pose is None and rclpy.ok():
            time.sleep(0.1)
            
        print("\n=============================================")
        print("  MARKER DETECTED! Starting Calibration...")
        print("  Ensure robot has space to move ~2cm safely.")
        print("=============================================")
        
        sign_x = self.test_axis('x', 0)
        sign_y = self.test_axis('y', 1)
        sign_z = self.test_axis('z', 2)
        
        print("\n=============================================")
        print("         CALIBRATION COMPLETE                ")
        print("=============================================")
        print("Update your pbvs_controller_config.yaml with:")
        print(f"    sign_x: {sign_x}")
        print(f"    sign_y: {sign_y}")
        print(f"    sign_z: {sign_z}")
        print("=============================================\n")
        
        # Exit program
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    wizard = PbvsSignWizard()
    
    # Run the wizard in a separate thread so ROS callbacks keep updating the pose
    wizard_thread = threading.Thread(target=wizard.run_wizard)
    wizard_thread.start()
    
    try:
        rclpy.spin(wizard)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        wizard_thread.join()

if __name__ == '__main__':
    main()