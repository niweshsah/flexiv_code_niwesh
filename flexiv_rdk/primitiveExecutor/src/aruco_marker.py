import cv2
import pyrealsense2 as rs
import numpy as np
import socket
import json

# --- NETWORK CONFIGURATION ---
ROBOT_IP = "192.168.10.6"  # Updated Flexiv Controller IP
UDP_PORT = 5005            # Shared Data port for C++ Framework
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- REALSENSE D455 SETUP ---
pipeline = rs.pipeline()
config = rs.config()

# Based on your device specs, 640x480 @ 30fps is supported for both Color and Depth
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming data pipelines and setup alignment (depth to color)
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

# --- ARUCO MARKER DICTIONARY ---
# Using standard 4x4 tag layouts. Change this if you printed a different dictionary!
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

print(f"[INFO] Intel RealSense D455 Vision Service Live.")
print(f"[INFO] Transmitting telemetry to {ROBOT_IP}:{UDP_PORT}...")

try:
    while True:
        # 1. Grab frames from RealSense
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        # Convert images to standard numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        
        # 2. Run ArUco Detection
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        # CHECK: Did we find *any* marker?
        if ids is not None and len(ids) > 0:
            # Pick the very first marker detected in the array
            target_idx = 0 
            detected_id = int(ids[target_idx][0])
            marker_corners = corners[target_idx][0] # Extracted [4x2] array of the 4 corners
            
            # 3. Sample exact central physical depth mapping
            center_x = int(np.mean(marker_corners[:, 0]))
            center_y = int(np.mean(marker_corners[:, 1]))
            depth_meters = depth_frame.get_distance(center_x, center_y)

            # Draw visual debug graphics on screen output
            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
            cv2.circle(color_image, (center_x, center_y), 5, (0, 255, 0), -1)
            cv2.putText(color_image, f"Tracking ID: {detected_id}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # 4. Construct the runtime metrics telemetry payload
            # NOTE: We hardcode "objName" to "aruco_marker" so it matches the YAML file natively
            telemetry_data = {
                "objName": "aruco_marker", 
                "detected": True,
                "currentFeaturePts": marker_corners.tolist(), # Current [x,y] coordinates
                "currentDepth": float(depth_meters)           # Accurate distance tracking
            }

            # 5. Blast JSON over UDP socket to the C++ Application
            payload = json.dumps(telemetry_data).encode('utf-8')
            sock.sendto(payload, (ROBOT_IP, UDP_PORT))
            
            print(f"[TRACKING] ID: {detected_id} | Depth: {depth_meters:.3f}m | Center: ({center_x}, {center_y})")
            
        else:
            # Send fallback payload so the robot knows the target is lost
            lost_payload = json.dumps({"objName": "aruco_marker", "detected": False}).encode('utf-8')
            sock.sendto(lost_payload, (ROBOT_IP, UDP_PORT))

        # Show view screen stream window
        cv2.imshow("D455 Eye-In-Hand Feed", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()