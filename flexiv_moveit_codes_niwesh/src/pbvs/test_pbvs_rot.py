#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
import time
import threading
import math

# Helper to convert Quaternions to Euler Angles without requiring external ROS packages
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

class PbvsRotationWizard(Node):
    def __init__(self):
        super().__init__('pbvs_rotation_wizard')
        
        # Ensure this matches your robot's End-Effector frame
        self.ee_frame = 'Rizon4s-063387_flange'
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/vision/target_pose', self.pose_callback, 10)
        
        self.current_euler = None
        
        # Test Parameters: 0.1 rad/s * 2.0s = 0.2 radians (~11.4 degrees of rotation)
        self.test_angular_velocity = 0.1 
        self.test_duration = 2.0 
        
        self.get_logger().info("Rotation Wizard Started. Waiting for ArUco marker...")

    def pose_callback(self, msg: PoseStamped):
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        
        try:
            transformed_pose = self.tf_buffer.transform(msg, self.ee_frame, rclpy.duration.Duration(seconds=0.1))
            q = transformed_pose.pose.orientation
            # Convert quaternion to Roll (Rx), Pitch (Ry), Yaw (Rz)
            self.current_euler = euler_from_quaternion(q.x, q.y, q.z, q.w)
        except Exception:
            pass 

    def publish_twist(self, rx=0.0, ry=0.0, rz=0.0):
        msg = TwistStamped()
        msg.header.frame_id = self.ee_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        # Note: We are setting ANGULAR velocity, not linear
        msg.twist.angular.x = rx
        msg.twist.angular.y = ry
        msg.twist.angular.z = rz
        self.twist_pub.publish(msg)

    def test_axis(self, axis_name, axis_index):
        print(f"\n--- TESTING {axis_name.upper()} AXIS ---")
        input(f"Press [ENTER] to rotate the robot ~11 degrees in +{axis_name.upper()}...")
        
        if self.current_euler is None:
            print("ERROR: No marker detected! Please ensure the ArUco marker is in view.")
            return 1
            
        # 1. Record starting orientation
        start_val = self.current_euler[axis_index]
        
        # 2. Command movement
        v = [0.0, 0.0, 0.0]
        v[axis_index] = self.test_angular_velocity
        
        t_end = time.time() + self.test_duration
        while time.time() < t_end:
            self.publish_twist(rx=v[0], ry=v[1], rz=v[2])
            time.sleep(0.01)
            
        # 3. Stop movement and wait for settling
        self.publish_twist(0.0, 0.0, 0.0)
        time.sleep(0.5)
        
        # 4. Record ending orientation
        end_val = self.current_euler[axis_index]
        
        # 5. Calculate Delta (Safely handling Pi to -Pi wrap-arounds)
        delta = end_val - start_val
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        
        delta_degrees = math.degrees(delta)
        print(f"[{axis_name.upper()}] Marker orientation changed by: {delta_degrees:.2f} degrees")
        
        # If commanding +W makes the marker's relative angle decrease, the frame is standard.
        if delta < 0:
            print(f"Result: Standard Alignment. sign_{axis_name} = 1")
            return 1
        else:
            print(f"Result: Reversed Alignment. sign_{axis_name} = -1")
            return -1

    def run_wizard(self):
        while self.current_euler is None and rclpy.ok():
            time.sleep(0.1)
            
        print("\n=============================================")
        print("  MARKER DETECTED! Starting Rotation Calib...")
        print("  WARNING: Robot will twist/tilt 11 degrees!")
        print("  Keep your hand near the E-STOP.")
        print("=============================================")
        
        sign_rx = self.test_axis('rx', 0)
        sign_ry = self.test_axis('ry', 1)
        sign_rz = self.test_axis('rz', 2)
        
        print("\n=============================================")
        print("         CALIBRATION COMPLETE                ")
        print("=============================================")
        print("Update your pbvs_controller_config.yaml with:")
        print(f"    sign_rx: {sign_rx}")
        print(f"    sign_ry: {sign_ry}")
        print(f"    sign_rz: {sign_rz}")
        print("=============================================\n")
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    wizard = PbvsRotationWizard()
    
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