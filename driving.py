from roboclaw import Roboclaw
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import signal
import atexit
from ultralytics import YOLO
import threading
import glob
import os
from collections import deque
import sys

# added encoder printing 
# equivalent of test_aug_30.py file

GLOBAL_ROBOT = None

def hard_motor_stop():
    """Force-stop motors immediately and close connection if possible."""
    try:
        if GLOBAL_ROBOT:
            try:
                GLOBAL_ROBOT.is_shutting_down = True
            except Exception:
                pass
        if GLOBAL_ROBOT and GLOBAL_ROBOT.connection_manager and GLOBAL_ROBOT.connection_manager.roboclaw:
            rc = GLOBAL_ROBOT.connection_manager.roboclaw
            addr = GLOBAL_ROBOT.connection_manager.address
            for _ in range(3):
                try:
                    rc.SpeedM1(addr, 0)
                    rc.SpeedM2(addr, 0)
                except Exception:
                    pass
                try:
                    rc._port.flush()
                except Exception:
                    pass
                time.sleep(0.02)
    except Exception:
        pass

def _emergency_stop():
    """Best-effort stop for motors on interpreter exit or signals"""
    try:
        if GLOBAL_ROBOT:
            # Hard stop first, then graceful ramp stop
            hard_motor_stop()
            GLOBAL_ROBOT.stop_motors()
    except Exception as e:
        print(f"Emergency stop error: {e}")

def _signal_handler(signum, frame):
    print(f"Signal {signum} received - performing emergency stop and exiting")
    try:
        hard_motor_stop()
    finally:
        _emergency_stop()
        sys.exit(0)

atexit.register(_emergency_stop)
signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)
# ADDED SHUTTING DOWN WHEN THE RASPBERRY PI IS SHUT DOWN
for _sig_name in ['SIGHUP', 'SIGQUIT', 'SIGABRT']:
    _sig = getattr(signal, _sig_name, None)
    if _sig is not None:
        try:
            signal.signal(_sig, _signal_handler)
        except Exception:
            pass

class RoboClawConnectionManager:
    """Manages RoboClaw connection with automatic reconnection"""
    
    def __init__(self, port="/dev/ttyACM0", baudrate=38400, address=0x80, heartbeat_interval=1.0):
        self.port = port
        self.baudrate = baudrate
        self.address = address
        self.heartbeat_interval = heartbeat_interval
        self.roboclaw = None
        self.connected = False
        self.heartbeat_thread = None
        self.stop_heartbeat = False
        
    def find_roboclaw_ports(self):
        """Find available RoboClaw ports"""
        ports = []
        for pattern in ["/dev/ttyACM*", "/dev/ttyUSB*"]:
            ports.extend(glob.glob(pattern))
        return sorted(ports)
    
    def test_roboclaw_connection(self, port):
        """Test if a port has a RoboClaw"""
        try:
            if not self.check_port_permissions(port):
                return False
                
            test_roboclaw = Roboclaw(port, self.baudrate)
            test_roboclaw.Open()
            
            # Test communication
            result, version = test_roboclaw.ReadVersion(self.address)
            if result:
                print(f"Found RoboClaw on {port}: {version}")
                test_roboclaw._port.close()
                return True
            else:
                test_roboclaw._port.close()
                return False
        except Exception as e:
            print(f"Error testing {port}: {e}")
            return False
    
    def check_port_permissions(self, port):
        """Check if we have read/write permissions on the port"""
        try:
            return os.access(port, os.R_OK | os.W_OK)
        except:
            return False
    
    def scan_and_connect(self):
        """Scan all ports and connect to first working RoboClaw"""
        ports = self.find_roboclaw_ports()
        print(f"Scanning ports: {ports}")
        
        for port in ports:
            if self.test_roboclaw_connection(port):
                self.port = port
                print(f"Selected port: {self.port}")
                return True
        return False
    
    def wait_for_connection(self, max_retries=None, retry_delay=2.0):
        """Wait for RoboClaw connection"""
        retry_count = 0
        
        while max_retries is None or retry_count < max_retries:
            try:
                # Try original port first
                if self.test_roboclaw_connection(self.port):
                    self.roboclaw = Roboclaw(self.port, self.baudrate)
                    self.roboclaw.Open()
                    self.connected = True
                    print(f"Connected to RoboClaw on {self.port}")
                    return True
                
                # If original port failed, scan for new port
                print(f"Port {self.port} failed, scanning for new port...")
                if self.scan_and_connect():
                    self.roboclaw = Roboclaw(self.port, self.baudrate)
                    self.roboclaw.Open()
                    self.connected = True
                    print(f"Connected to RoboClaw on {self.port}")
                    return True
                
                print(f"Connection attempt {retry_count + 1} failed, retrying in {retry_delay}s...")
                time.sleep(retry_delay)
                retry_count += 1
                
            except Exception as e:
                print(f"Connection error: {e}")
                retry_count += 1
                time.sleep(retry_delay)
        
        print("Failed to connect to RoboClaw")
        return False
    
    def start_heartbeat(self):
        """Start heartbeat monitoring thread"""
        if self.heartbeat_thread is None or not self.heartbeat_thread.is_alive():
            self.stop_heartbeat = False
            self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self.heartbeat_thread.start()
            print("Heartbeat monitoring started")
    
    def _heartbeat_loop(self):
        """Heartbeat monitoring loop"""
        while not self.stop_heartbeat:
            try:
                if self.roboclaw and self.connected:
                    result, version = self.roboclaw.ReadVersion(self.address)
                    if not result:
                        print("RoboClaw heartbeat failed - connection lost")
                        self.connected = False
                        self.roboclaw._port.close()
                        self.roboclaw = None
                time.sleep(self.heartbeat_interval)
            except Exception as e:
                print(f"Heartbeat error: {e}")
                self.connected = False
                if self.roboclaw:
                    try:
                        self.roboclaw._port.close()
                    except:
                        pass
                    self.roboclaw = None
                time.sleep(self.heartbeat_interval)
    
    def ensure_connection(self):
        """Ensure connection is active, reconnect if needed"""
        if not self.connected or not self.roboclaw:
            print("RoboClaw disconnected, attempting to reconnect...")
            return self.wait_for_connection()
        return True
    
    def is_connected(self):
        """Check if currently connected"""
        return self.connected and self.roboclaw is not None
    
    def get_roboclaw(self):
        """Get RoboClaw instance if connected"""
        return self.roboclaw if self.connected else None
    
    def close(self):
        """Close connection and stop heartbeat"""
        self.stop_heartbeat = True
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=1.0)
        
        if self.roboclaw:
            try:
                self.roboclaw._port.close()
            except:
                pass
            self.roboclaw = None
        
        self.connected = False
        print("RoboClaw connection closed")

class PeopleFollowingRobot:
    """People following robot with obstacle avoidance"""
    
    def __init__(self):
        # RoboClaw connection
        self.connection_manager = RoboClawConnectionManager()
        
        # Motor control parameters - increased speeds for faster movement
        self.FORWARD_SPEED = 225         # increased from 200
        self.TURN_SPEED = 150            # Turning speed (doubled from 75)
        self.BACKUP_SPEED = 150          # Backup speed (doubled from 75)
        self.SEARCH_SPEED = 80           # Search turning speed (doubled from 40)
        self.RAMP_STEP_DELAY_S = 0.04    # decreased from 0.05
        
        # Obstacle avoidance
        self.OBSTACLE_METERS = 0.8       # increased from 0.5
        self.SLOW_METERS = 1.5           # increased from 1.0
        self.OBSTACLE_CLEAR_METERS = 1.0  # hysteresis clear distance (> OBSTACLE_METERS)
        self.OBSTACLE_BACKUP_METERS = 0.4 # if extremely close, back up instead of pivot
        
        # Person following distances (inches converted to meters)
        INCH = 0.0254
        self.BACK_AWAY_TRIGGER_M = 2 * INCH    # 0-2 inches -> back away
        self.BACK_AWAY_TARGET_M = 4 * INCH     # back up until 4 inches away
        self.STOP_MIN_M = 3 * INCH             # 3-6 inches -> full stop
        self.STOP_MAX_M = 6 * INCH
        self.FOLLOW_MIN_M = 7 * INCH           # >=7 inches -> follow
        self.PERSON_SLOW_DISTANCE = 0.5        # Slow down when approaching closer than 0.5m
        
        # Movement smoothing
        self.current_left_speed = 0
        self.current_right_speed = 0
        # Person centering behavior params
        self.CENTER_TOLERANCE_PX = 200   # ±200px → 400px wide tolerance zone
        self.CENTERING_TURN_SPEED = 80   # Slow and stable turning for centering
        # 5-frame smoothing buffers
        self._position_x_history = deque(maxlen=5)
        self._distance_history = deque(maxlen=5)
        self.obstacle_distance_history = deque(maxlen=5)
        
        # YOLO model
        self.yolo_model = YOLO('yolov8n.pt')
        self.confidence_threshold = 0.5
        
        # RealSense setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # State tracking
        self.person_detected = False
        self.last_person_time = 0
        self.searching = False
        self.avoiding_obstacle = False
        self.backing_to_four = False  # When triggered, back up until 4 inches is reached
        # Centering oscillation control
        self.centering_direction = 0  # -1 left, 0 straight, 1 right
        self.last_center_switch_time = 0.0
        self.CENTER_SWITCH_COOLDOWN_S = 0.4
        self.CENTER_SWITCH_MARGIN_PX = 40
        # Shutdown flag prevents further motor commands during exit
        self.is_shutting_down = False
        # Remember which side the person was last seen on: -1 left, 0 center/unknown, 1 right
        self.last_seen_direction = 0
        
    def setup(self):
        """Initialize robot systems"""
        print("Setting up People Following Robot...")
        
        # Connect to RoboClaw
        if not self.connection_manager.wait_for_connection():
            print("Failed to connect to RoboClaw")
            return False
        
        # Start heartbeat monitoring
        self.connection_manager.start_heartbeat()
        
        # Start RealSense
        try:
            self.pipeline.start(self.config)
            print("RealSense camera started")
        except Exception as e:
            print(f"Failed to start RealSense: {e}")
            return False
        
        print("Robot setup complete!")
        return True
    
    def print_encoder_values(self):
        # added printing encoder values to the driving part
        rc = self.connection_manager.get_roboclaw()
        if not rc:
            return
        try:
            enc1 = rc.ReadEncM1(self.connection_manager.address)
            enc2 = rc.ReadEncM2(self.connection_manager.address)
            if enc1[0]:
                m1_text = f"M1 Ticks: {enc1[1]}, Status: {enc1[2]}"
            else:
                m1_text = "M1: read failed"
            if enc2[0]:
                m2_text = f"M2 Ticks: {enc2[1]}, Status: {enc2[2]}"
            else:
                m2_text = "M2: read failed"
            print(f"ENCODERS -> {m1_text} | {m2_text}")
        except Exception as e:
            print(f"encoder read error: {e}")

    def set_motor_speeds(self, left_speed, right_speed):
        """Set motor speeds with safety checks"""
        if getattr(self, 'is_shutting_down', False):
            return False
        if not self.connection_manager.ensure_connection():
            print("Cannot set motor speeds - no connection")
            return False
        
        roboclaw = self.connection_manager.get_roboclaw()
        if not roboclaw:
            print("No RoboClaw instance available")
            return False
        
        try:
            # M1: left motor (positive), M2: right motor (negative) for forward movement
            roboclaw.SpeedM1(self.connection_manager.address, left_speed)
            roboclaw.SpeedM2(self.connection_manager.address, -right_speed)
            return True
        except Exception as e:
            print(f"Error setting motor speeds: {e}")
            return False
    
    def stop_motors(self):
        """Stop both motors smoothly"""
        self.is_shutting_down = True
        # Send multiple zero commands to ensure controller latches stop
        roboclaw = self.connection_manager.get_roboclaw()
        if roboclaw:
            for _ in range(3):
                try:
                    roboclaw.SpeedM1(self.connection_manager.address, 0)
                    roboclaw.SpeedM2(self.connection_manager.address, 0)
                except Exception:
                    pass
                time.sleep(0.02)
        print("Stopping motors smoothly")
        self.ramp_to_speed(0, 0, duration=0.3)
        self.current_left_speed = 0
        self.current_right_speed = 0
    
    def calculate_ramp_duration(self, target_left, target_right):
        """Calculate optimal ramp duration based on speed change magnitude"""
        # Calculate total speed change
        left_change = abs(target_left - self.current_left_speed)
        right_change = abs(target_right - self.current_right_speed)
        total_change = left_change + right_change
        
        # Check for direction changes
        direction_change = False
        if (self.current_left_speed > 0 and target_left < 0) or (self.current_left_speed < 0 and target_left > 0):
            direction_change = True
        if (self.current_right_speed > 0 and target_right < 0) or (self.current_right_speed < 0 and target_right > 0):
            direction_change = True
        
        if direction_change:
            # Direction changes always need longer ramps for safety
            return 0.3
        elif total_change < 30:  # Very small adjustment
            return 0.02  # Almost instant
        elif total_change < 80:  # Small adjustment
            return 0.05  # Very fast ramp
        elif total_change < 150:  # Medium adjustment
            return 0.1   # Fast ramp
        else:  # Large change
            return 0.2   # Moderate ramp
    
    def ramp_to_speed(self, target_left, target_right, duration=None):
        """Smoothly ramp to target speeds with adaptive duration"""
        if getattr(self, 'is_shutting_down', False):
            # Do not ramp further during shutdown
            try:
                roboclaw = self.connection_manager.get_roboclaw()
                if roboclaw:
                    roboclaw.SpeedM1(self.connection_manager.address, 0)
                    roboclaw.SpeedM2(self.connection_manager.address, 0)
            except Exception:
                pass
            self.current_left_speed = 0
            self.current_right_speed = 0
            return
        if target_left == self.current_left_speed and target_right == self.current_right_speed:
            return
        
        # Calculate optimal ramp duration if not specified
        if duration is None:
            duration = self.calculate_ramp_duration(target_left, target_right)
        
        # Check if we need to change direction
        direction_change = False
        if (self.current_left_speed > 0 and target_left < 0) or (self.current_left_speed < 0 and target_left > 0):
            direction_change = True
        if (self.current_right_speed > 0 and target_right < 0) or (self.current_right_speed < 0 and target_right > 0):
            direction_change = True
        
        if direction_change:
            # Always ramp through zero for direction changes
            print(f"Direction change detected - ramping through zero")
            # Phase 1: Ramp to zero
            self.ramp_to_speed(0, 0, duration/2)
            # Phase 2: Ramp from zero to target
            self.ramp_to_speed(target_left, target_right, duration/2)
        else:
            # Normal ramping (same direction)
            steps = max(1, int(duration / self.RAMP_STEP_DELAY_S))
            left_step = (target_left - self.current_left_speed) / steps
            right_step = (target_right - self.current_right_speed) / steps
            
            for i in range(steps):
                left_speed = int(self.current_left_speed + left_step * (i + 1))
                right_speed = int(self.current_right_speed + right_step * (i + 1))
                
                if self.set_motor_speeds(left_speed, right_speed):
                    self.current_left_speed = left_speed
                    self.current_right_speed = right_speed
                
                time.sleep(self.RAMP_STEP_DELAY_S)
    
    def get_obstacle_distance(self, depth_frame):
        """Scan 21x11 pixel grid around center for obstacles"""
        if not depth_frame:
            return float('inf')
        
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        
        # Scan 21x11 grid around center
        min_distance = float('inf')
        for i in range(21):
            for j in range(11):
                x = (width // 2) - 10 + i
                y = (height // 2) - 5 + j
                distance = depth_frame.get_distance(x, y)
                if distance > 0:
                    min_distance = min(distance, min_distance)
        
        return min_distance
    
    def detect_person(self, color_frame, depth_frame):
        """Detect person using YOLO and return position/distance"""
        if not color_frame or not depth_frame:
            return None
        
        # Convert to numpy array for YOLO
        color_image = np.asanyarray(color_frame.get_data())
        
        # Run YOLO detection
        results = self.yolo_model(color_image, verbose=False)
        
        closest_person = None
        min_distance = float('inf')
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Check if it's a person (class 0)
                    if int(box.cls[0]) == 0 and box.conf[0] > self.confidence_threshold:
                        # Get bounding box
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2
                        
                        # Calculate distance at person center
                        distance = depth_frame.get_distance(int(center_x), int(center_y))
                        if distance > 0 and distance < min_distance:
                            min_distance = distance
                            closest_person = {
                                'center_x': center_x,
                                'center_y': center_y,
                                'distance': distance,
                                'confidence': float(box.conf[0])
                            }
        
        # Apply 5-frame smoothing on x-position and distance
        if closest_person is not None:
            self._position_x_history.append(float(closest_person['center_x']))
            self._distance_history.append(float(closest_person['distance']))
            smoothed_x = float(np.mean(self._position_x_history)) if len(self._position_x_history) > 0 else float(closest_person['center_x'])
            smoothed_distance = float(np.mean(self._distance_history)) if len(self._distance_history) > 0 else float(closest_person['distance'])
            closest_person['center_x'] = smoothed_x
            closest_person['distance'] = smoothed_distance
        else:
            # Clear histories when person not detected
            self._position_x_history.clear()
            self._distance_history.clear()

        return closest_person
    
    def calculate_movement(self, person_info, obstacle_distance):
        """Calculate movement based on person position and obstacles"""
        left_speed = 0
        right_speed = 0
        
        # Priority 1: Obstacle avoidance (non-person obstacles) with hysteresis & backup
        if obstacle_distance <= self.OBSTACLE_METERS:
            if not self.avoiding_obstacle:
                print(f"OBSTACLE DETECTED: {obstacle_distance:.2f}m - STARTING OBSTACLE AVOIDANCE")
                self.avoiding_obstacle = True
            # Disable centering while avoiding
            self.centering_direction = 0
            if obstacle_distance <= self.OBSTACLE_BACKUP_METERS:
                # Very close: back straight away
                left_speed = -self.BACKUP_SPEED
                right_speed = -self.BACKUP_SPEED
                print(f"  Avoiding obstacle: BACKING UP (distance: {obstacle_distance:.2f}m)")
            else:
                # Pivot left in place
                left_speed = -self.TURN_SPEED
                right_speed = -self.TURN_SPEED
                print(f"  Avoiding obstacle: TURNING LEFT (distance: {obstacle_distance:.2f}m)")
            return left_speed, right_speed
        elif self.avoiding_obstacle and obstacle_distance < self.OBSTACLE_CLEAR_METERS:
            # Keep avoiding until we reach clear threshold
            self.centering_direction = 0
            left_speed = -self.TURN_SPEED
            right_speed = -self.TURN_SPEED
            print(f"  Avoiding obstacle: CONTINUE TURN LEFT (distance: {obstacle_distance:.2f}m)")
            return left_speed, right_speed
        elif self.avoiding_obstacle and obstacle_distance >= self.OBSTACLE_CLEAR_METERS:
            print(f"OBSTACLE CLEARED: {obstacle_distance:.2f}m - RESUMING NORMAL OPERATION")
            self.avoiding_obstacle = False
        
        # Priority 2: Person following
        if person_info:
            person_x = person_info['center_x']
            person_distance = person_info['distance']
            
            # Calculate lateral error (how far person is from center)
            image_center = 320  # 640/2
            lateral_error = person_x - image_center
            
            print(f"PERSON DETECTED:")
            print(f"  Distance: {person_distance:.2f}m")
            print(f"  Position: {person_x:.1f}px from center (error: {lateral_error:.1f}px)")
            # Update last seen side for search behavior if the person is meaningfully off-center
            if abs(lateral_error) > self.CENTER_SWITCH_MARGIN_PX:
                self.last_seen_direction = 1 if lateral_error > 0 else -1
            
            # Distance control with new thresholds and hysteresis
            # 0-2 inches: trigger backing behavior to 4 inches
            if person_distance <= self.BACK_AWAY_TRIGGER_M:
                print(f"  Too close (<=2in) - backing up to 4in distance")
                self.backing_to_four = True
                base_speed = -self.BACKUP_SPEED
                left_speed = base_speed
                right_speed = base_speed
                # Disable centering while backing
                self.centering_direction = 0
                return left_speed, right_speed

            # If we are in backing_to_four mode, keep backing until target reached, then stop
            if self.backing_to_four:
                if person_distance < self.BACK_AWAY_TARGET_M:
                    print(f"  Backing... distance {person_distance:.3f}m; target 4in")
                    base_speed = -self.BACKUP_SPEED
                    left_speed = base_speed
                    right_speed = base_speed
                    self.centering_direction = 0
                    return left_speed, right_speed
                else:
                    print("  Reached 4in - stopping")
                    self.backing_to_four = False
                    left_speed = 0
                    right_speed = 0
                    self.centering_direction = 0
                    return left_speed, right_speed

            # 3-6 inches: full stop (also treat 2-3 inches as stop for safety)
            if person_distance < self.STOP_MIN_M or (self.STOP_MIN_M <= person_distance <= self.STOP_MAX_M):
                print(f"  Within stop zone (<~6in) - FULL STOP")
                left_speed = 0
                right_speed = 0
                self.centering_direction = 0
                return left_speed, right_speed

            # 6-7 inches: hold stop until >= 7 inches to avoid oscillation
            if person_distance < self.FOLLOW_MIN_M:
                print(f"  In hysteresis band (6-7in) - holding STOP")
                left_speed = 0
                right_speed = 0
                self.centering_direction = 0
                return left_speed, right_speed

            else:
                # > ~8 inches: approach with slow-down near 0.5m and allow turning
                if person_distance <= self.PERSON_SLOW_DISTANCE:
                    print(f"  Close (<~0.5m) - slowing down")
                    base_speed = self.FORWARD_SPEED // 2
                else:
                    print(f"  Far (>=~0.5m) - approaching at normal speed")
                    base_speed = self.FORWARD_SPEED

                # Turning control with hysteresis and cooldown to prevent oscillation
                now = time.time()
                intended_direction = 0
                if lateral_error > self.CENTER_TOLERANCE_PX:
                    intended_direction = 1
                elif lateral_error < -self.CENTER_TOLERANCE_PX:
                    intended_direction = -1

                can_switch = (now - self.last_center_switch_time) >= self.CENTER_SWITCH_COOLDOWN_S

                if self.centering_direction == 0:
                    if intended_direction != 0 and can_switch:
                        self.centering_direction = intended_direction
                        self.last_center_switch_time = now
                else:
                    if intended_direction == 0:
                        if can_switch:
                            self.centering_direction = 0
                            self.last_center_switch_time = now
                    elif intended_direction != self.centering_direction:
                        if can_switch and abs(lateral_error) >= (self.CENTER_TOLERANCE_PX + self.CENTER_SWITCH_MARGIN_PX):
                            self.centering_direction = intended_direction
                            self.last_center_switch_time = now

                if self.centering_direction > 0:
                    turn_speed = self.CENTERING_TURN_SPEED
                    print(f"  Centering: turning right (sticky)")
                    left_speed = base_speed + turn_speed
                    right_speed = base_speed - turn_speed
                elif self.centering_direction < 0:
                    turn_speed = self.CENTERING_TURN_SPEED
                    print(f"  Centering: turning left (sticky)")
                    left_speed = base_speed - turn_speed
                    right_speed = base_speed + turn_speed
                else:
                    print(f"  Centering: within tolerance - holding heading")
                    left_speed = base_speed
                    right_speed = base_speed
            
            # Ensure speeds are within limits
            max_speed = self.FORWARD_SPEED * 2
            left_speed = max(-max_speed, min(max_speed, left_speed))
            right_speed = max(-max_speed, min(max_speed, right_speed))
            
            print(f"  Final speeds: Left={left_speed}, Right={right_speed}")
            
        else:
            # No person detected - continue forward while searching toward last seen side
            if not self.searching:
                self.searching = True
                print("No person detected - continuing forward while searching")
            
            if self.last_seen_direction > 0:
                # Last seen to the right → bias search right
                left_speed = self.FORWARD_SPEED + self.SEARCH_SPEED
                right_speed = self.FORWARD_SPEED - self.SEARCH_SPEED
                print(f"Searching: last seen on RIGHT - turning right slowly")
            elif self.last_seen_direction < 0:
                # Last seen to the left → bias search left
                left_speed = self.FORWARD_SPEED - self.SEARCH_SPEED
                right_speed = self.FORWARD_SPEED + self.SEARCH_SPEED
                print(f"Searching: last seen on LEFT - turning left slowly")
            else:
                # Unknown → default gentle left search
                left_speed = self.FORWARD_SPEED - self.SEARCH_SPEED
                right_speed = self.FORWARD_SPEED + self.SEARCH_SPEED
                print(f"Searching: no last seen side - turning left slowly")
            
            # Reset centering state when no person
            self.centering_direction = 0
        
        return left_speed, right_speed
    
    def draw_obstacle_zone(self, image, depth_frame):
        """Draw the 21x11 obstacle detection grid on the image"""
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        # Draw the scanning grid
        for i in range(21):
            for j in range(11):
                x = center_x - 10 + i
                y = center_y - 5 + j
                
                if 0 <= x < width and 0 <= y < height:
                    # Get distance at this point
                    distance = depth_frame.get_distance(x, y)
                    if distance > 0:
                        # Color code based on distance
                        if distance <= self.OBSTACLE_METERS:
                            color = (0, 0, 255)  # Red for close obstacles
                        elif distance <= self.SLOW_METERS:
                            color = (0, 255, 255)  # Yellow for slow zone
                        else:
                            color = (0, 255, 0)  # Green for safe distance
                        
                        cv2.circle(image, (x, y), 2, color, -1)
        
        # Draw center crosshair
        cv2.line(image, (center_x-10, center_y), (center_x+10, center_y), (255, 255, 255), 2)
        cv2.line(image, (center_x, center_y-10), (center_x, center_y+10), (255, 255, 255), 2)
    
    def draw_person_info(self, image, person_info):
        """Draw person detection information on the image"""
        x, y = int(person_info['center_x']), int(person_info['center_y'])
        distance = person_info['distance']
        confidence = person_info['confidence']
        
        # Draw bounding box (approximate)
        box_size = 50
        cv2.rectangle(image, (x-box_size//2, y-box_size//2), (x+box_size//2, y+box_size//2), (0, 255, 0), 2)
        
        # Draw center point
        cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
        
        # Draw distance and confidence text
        text = f"Person: {distance:.2f}m ({confidence:.2f})"
        cv2.putText(image, text, (x-box_size//2, y-box_size//2-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    def draw_status_info(self, image, obstacle_distance, person_info):
        """Draw status information on the image"""
        height, width = image.shape[:2]
        
        # Background for text
        cv2.rectangle(image, (10, 10), (400, 120), (0, 0, 0), -1)
        cv2.rectangle(image, (10, 10), (400, 120), (255, 255, 255), 2)
        
        # Status text
        y_offset = 30
        cv2.putText(image, f"Obstacle Distance: {obstacle_distance:.2f}m", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        y_offset += 25
        if person_info:
            cv2.putText(image, f"Person Detected: YES", (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_offset += 25
            cv2.putText(image, f"Person Distance: {person_info['distance']:.2f}m", (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(image, f"Person Detected: NO", (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        y_offset += 25
        if self.avoiding_obstacle:
            cv2.putText(image, f"Status: AVOIDING OBSTACLE", (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        elif self.searching:
            cv2.putText(image, f"Status: SEARCHING", (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        else:
            cv2.putText(image, f"Status: FOLLOWING", (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    def run(self):
        """Main robot control loop"""
        print("Starting people following robot...")
        print("Press 'q' to quit the viewer")
        
        try:
            while True:
                # Get camera frames
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    print("No camera frames available")
                    time.sleep(0.1)
                    continue
                
                # Convert frames to numpy arrays for display
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Create display image
                display_image = color_image.copy()
                
                # Check for obstacles (non-person) with smoothing
                obstacle_distance_raw = self.get_obstacle_distance(depth_frame)
                self.obstacle_distance_history.append(obstacle_distance_raw)
                obstacle_distance = float(np.median(self.obstacle_distance_history)) if len(self.obstacle_distance_history) > 0 else obstacle_distance_raw
                
                # Detect person
                person_info = self.detect_person(color_frame, depth_frame)
                
                # Draw obstacle detection zone (21x11 grid)
                self.draw_obstacle_zone(display_image, depth_frame)
                
                # Draw person detection info
                if person_info:
                    self.draw_person_info(display_image, person_info)
                
                # Draw status information
                self.draw_status_info(display_image, obstacle_distance, person_info)
                
                # Update person detection state
                if person_info:
                    self.person_detected = True
                    self.last_person_time = time.time()
                    self.searching = False
                else:
                    # Person lost if not seen for 2 seconds
                    if time.time() - self.last_person_time > 2.0:
                        self.person_detected = False
                
                # Calculate movement
                left_speed, right_speed = self.calculate_movement(person_info, obstacle_distance)
                
                # Apply movement with adaptive smoothing
                if left_speed != self.current_left_speed or right_speed != self.current_right_speed:
                    print(f"MOVEMENT: Left={left_speed}, Right={right_speed}")
                    # Use adaptive ramping - small changes are almost instant, large changes are smooth
                    self.ramp_to_speed(left_speed, right_speed)
                
                # print encoder values each loop iteration
                self.print_encoder_values()
                
                # Display the image
                cv2.imshow('Robot Vision - People Following & Obstacle Avoidance', display_image)
                
                # Check for key press to quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Quit requested by user")
                    break
                
                # Small delay for control loop
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("Stopping robot...")
        except Exception as e:
            print(f"Error in control loop: {e}")
        finally:
            self.stop_motors()
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        self.is_shutting_down = True
        self.stop_motors()
        self.connection_manager.close()
        self.pipeline.stop()
        print("Cleanup complete")

def main():
    """Main function"""
    global GLOBAL_ROBOT
    robot = PeopleFollowingRobot()
    GLOBAL_ROBOT = robot
    
    if robot.setup():
        try:
            robot.run()
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            robot.cleanup()
    else:
        print("Failed to setup robot")

if __name__ == "__main__":
    main()
