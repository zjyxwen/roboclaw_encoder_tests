from roboclaw import Roboclaw
import time
import signal
import atexit
import threading
import glob
import os
import sys


GLOBAL_ROBOT = None

# manual driving robot, using WASD commands to move the robot
# printing encoder values to the terminal constantly

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
# Handle additional shutdown-related signals when available
for _sig_name in ['SIGHUP', 'SIGQUIT', 'SIGABRT']:
    _sig = getattr(signal, _sig_name, None)
    if _sig is not None:
        try:
            signal.signal(_sig, _signal_handler)
        except Exception:
            pass


class RoboClawConnectionManager:
    """Manages RoboClaw connection with automatic reconnection"""

    def __init__(self, port="/dev/ttyACM0", baudrate=38400, address=0x80, heartbeat_interval=0.5):
        self.port = port
        self.baudrate = baudrate
        self.address = address
        self.heartbeat_interval = heartbeat_interval
        self.roboclaw = None
        self.connected = False
        self.heartbeat_thread = None
        self.stop_heartbeat = False
        # Async reconnect
        self.reconnect_thread = None
        self.reconnecting = False
        self._conn_lock = threading.Lock()

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

            # Fast probe: low timeout, single retry
            test_roboclaw = Roboclaw(port, self.baudrate, timeout=0.15, retries=1)
            test_roboclaw.Open()

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
        except Exception:
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

    def wait_for_connection(self, max_retries=None, retry_delay=0.5):
        """Wait for RoboClaw connection"""
        retry_count = 0

        while max_retries is None or retry_count < max_retries:
            try:
                if self.test_roboclaw_connection(self.port):
                    self.roboclaw = Roboclaw(self.port, self.baudrate, timeout=0.2)
                    self.roboclaw.Open()
                    self.connected = True
                    print(f"Connected to RoboClaw on {self.port}")
                    return True

                print(f"Port {self.port} failed, scanning for new port...")
                if self.scan_and_connect():
                    self.roboclaw = Roboclaw(self.port, self.baudrate, timeout=0.2)
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
                        with self._conn_lock:
                            self.connected = False
                            try:
                                if self.roboclaw and getattr(self.roboclaw, "_port", None):
                                    self.roboclaw._port.close()
                            except Exception:
                                pass
                            self.roboclaw = None
                        self.start_async_reconnect()
                time.sleep(self.heartbeat_interval)
            except Exception as e:
                print(f"Heartbeat error: {e}")
                with self._conn_lock:
                    self.connected = False
                    if self.roboclaw:
                        try:
                            self.roboclaw._port.close()
                        except Exception:
                            pass
                        self.roboclaw = None
                self.start_async_reconnect()
                time.sleep(self.heartbeat_interval)

    def start_async_reconnect(self):
        """Start background reconnect if not already running"""
        if self.reconnecting:
            return
        self.reconnecting = True
        def _worker():
            try:
                for delay in [0.1, 0.2, 0.3, 0.5]:
                    if self.stop_heartbeat:
                        break
                    if self.wait_for_connection(max_retries=1, retry_delay=delay):
                        break
                while not self.stop_heartbeat and not self.connected:
                    self.wait_for_connection(max_retries=1, retry_delay=0.5)
                    if not self.connected:
                        time.sleep(0.5)
            finally:
                self.reconnecting = False
        self.reconnect_thread = threading.Thread(target=_worker, daemon=True)
        self.reconnect_thread.start()

    def ensure_connection(self):
        """Ensure connection is active, reconnect if needed"""
        if not self.connected or not self.roboclaw:
            print("RoboClaw disconnected, starting async reconnect...")
            self.start_async_reconnect()
            return False
        return True

    def is_connected(self):
        return self.connected and self.roboclaw is not None

    def get_roboclaw(self):
        return self.roboclaw if self.connected else None

    def close(self):
        """Close connection and stop heartbeat"""
        self.stop_heartbeat = True
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=1.0)
        if self.reconnect_thread and self.reconnect_thread.is_alive():
            self.reconnect_thread.join(timeout=1.0)

        if self.roboclaw:
            try:
                self.roboclaw._port.close()
            except Exception:
                pass
            self.roboclaw = None

        self.connected = False
        print("RoboClaw connection closed")


class ManualDriveRobot:
    """Manual driving robot using WASD commands"""

    def __init__(self):
        self.connection_manager = RoboClawConnectionManager()

        # Speed and timing calibration
        # Adjust these constants to match your robot's actual distance per time
        self.FORWARD_SPEED = 500  # nominal forward speed
        self.TURN_SPEED = 250     # nominal in-place turn speed
        self.SECONDS_PER_FOOT = 1.0  # time at FORWARD_SPEED to travel ~1 foot (tune as needed)
        self.SPIN_SECONDS_360 = 1.0  # time at TURN_SPEED to rotate exactly 360° (tune as needed)
        self.SPIN_SECONDS_90 = self.SPIN_SECONDS_360 / 4.0  # time for 90° turn

        self.is_shutting_down = False
        self._enc_thread = None
        self._enc_stop = False

    # printing encoder values to the terminal constantly
    def print_encoder_values(self):
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

    def _start_encoder_printer(self, interval=0.25):
        if self._enc_thread and self._enc_thread.is_alive():
            return
        self._enc_stop = False
        def _worker():
            while not self._enc_stop and not self.is_shutting_down:
                self.print_encoder_values()
                time.sleep(interval)
        self._enc_thread = threading.Thread(target=_worker, daemon=True)
        self._enc_thread.start()

    def setup(self):
        print("Setting up Manual Drive Robot...")
        if not self.connection_manager.wait_for_connection():
            print("Failed to connect to RoboClaw")
            return False
        self.connection_manager.start_heartbeat()
        print("Manual Drive setup complete!")
        # start periodic encoder printing
        self._start_encoder_printer()
        return True

    def run_pre_drive_tests(self):
        rc = self.connection_manager.get_roboclaw()
        addr = self.connection_manager.address
        if not rc:
            print("Pre-drive tests skipped: no RoboClaw connection")
            return
        print("\n================ PRE-DRIVE TESTS ================")
        try:
            result, version = rc.ReadVersion(addr)
            print("Success:", result)
            print("Version:", version)
        except Exception as e:
            print(f"ReadVersion error: {e}")
        try:
            status, volts = rc.ReadMainBatteryVoltage(addr)
            if status:
                print("Battery voltage:", volts / 10.0, "V")
            else:
                print("Battery voltage read failed")
        except Exception as e:
            print(f"Battery voltage read error: {e}")

        print("Trying DutyM1...")
        try:
            for i in range(20):
                if self.is_shutting_down:
                    break
                rc.DutyM1(addr, 819 * i)
                rc.DutyM2(addr, -819 * i)
                try:
                    s1 = rc.ReadSpeedM1(addr)[1]
                    s2 = rc.ReadSpeedM2(addr)[1]
                    print(s1 + s2)
                except Exception:
                    pass
                time.sleep(0.05)
            time.sleep(5)
            for i in range(20):
                if self.is_shutting_down:
                    break
                rc.DutyM1(addr, 819 * (20 - i))
                rc.DutyM2(addr, -819 * (20 - i))
                try:
                    s1 = rc.ReadSpeedM1(addr)[1]
                    s2 = rc.ReadSpeedM2(addr)[1]
                    print(s1 + s2)
                except Exception:
                    pass
                time.sleep(0.05)
            rc.DutyM1(addr, 0)
            rc.DutyM2(addr, 0)
            print("Done.")
        except Exception as e:
            print(f"Duty test error: {e}")

        time.sleep(5)

        print("Trying Speed")
        try:
            for i in range(20):
                if self.is_shutting_down:
                    break
                rc.SpeedM1(addr, 250 * i)
                rc.SpeedM2(addr, -250 * i)
                try:
                    s1 = rc.ReadSpeedM1(addr)[1]
                    s2 = rc.ReadSpeedM2(addr)[1]
                    print(s1 + s2)
                except Exception:
                    pass
                time.sleep(0.05)
            time.sleep(5)
            for i in range(20):
                if self.is_shutting_down:
                    break
                rc.SpeedM1(addr, 250 * (20 - i))
                rc.SpeedM2(addr, -250 * (20 - i))
                try:
                    s1 = rc.ReadSpeedM1(addr)[1]
                    s2 = rc.ReadSpeedM2(addr)[1]
                    print(s1 + s2)
                except Exception:
                    pass
                time.sleep(0.05)
            rc.SpeedM1(addr, 0)
            rc.SpeedM2(addr, 0)
            print("Done")
        except Exception as e:
            print(f"Speed test error: {e}")

        print("================ PRE-DRIVE TESTS COMPLETE ================\n")
        print("================ MANUAL DRIVING AVAILABLE ================")

    def set_motor_speeds(self, left_speed, right_speed):
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
            roboclaw.SpeedM1(self.connection_manager.address, left_speed)
            roboclaw.SpeedM2(self.connection_manager.address, -right_speed)
            return True
        except Exception as e:
            print(f"Error setting motor speeds: {e}")
            self.connection_manager.connected = False
            return False

    def stop_motors(self):
        self.is_shutting_down = True
        roboclaw = self.connection_manager.get_roboclaw()
        if roboclaw:
            for _ in range(3):
                try:
                    roboclaw.SpeedM1(self.connection_manager.address, 0)
                    roboclaw.SpeedM2(self.connection_manager.address, 0)
                except Exception:
                    pass
                time.sleep(0.02)
        print("Motors stopped")

    def move_forward_one_foot(self):
        print("Command: W (forward 1 ft)")
        if self.set_motor_speeds(self.FORWARD_SPEED, self.FORWARD_SPEED):
            time.sleep(self.SECONDS_PER_FOOT)
        self.set_motor_speeds(0, 0)

    def move_backward_one_foot(self):
        print("Command: S (backward 1 ft)")
        if self.set_motor_speeds(-self.FORWARD_SPEED, -self.FORWARD_SPEED):
            time.sleep(self.SECONDS_PER_FOOT)
        self.set_motor_speeds(0, 0)

    def spin_left(self):
        print("Command: A (spin left)")
        # In-place left pivot: left negative, right negative (maps to M2 positive)
        if self.set_motor_speeds(-self.TURN_SPEED, self.TURN_SPEED):
            time.sleep(self.SPIN_SECONDS_90)
        self.set_motor_speeds(0, 0)

    def spin_right(self):
        print("Command: D (spin right)")
        # In-place right pivot: left positive, right positive
        if self.set_motor_speeds(self.TURN_SPEED, -self.TURN_SPEED):
            time.sleep(self.SPIN_SECONDS_90)
        self.set_motor_speeds(0, 0)

    def run(self):
        print("Manual drive ready. Type commands then press Enter:")
        print("  W = forward 1 ft, S = backward 1 ft, A = spin left, D = spin right, Q = quit")
        try:
            while True:
                cmd = input("> ").strip().upper()
                if not cmd:
                    continue
                if cmd == 'Q':
                    print("Quit requested by user")
                    break
                elif cmd == 'W':
                    self.move_forward_one_foot()
                elif cmd == 'S':
                    self.move_backward_one_foot()
                elif cmd == 'A':
                    self.spin_left()
                elif cmd == 'D':
                    self.spin_right()
                else:
                    print("Unknown command. Use W/S/A/D or Q to quit.")
        except KeyboardInterrupt:
            print("Stopping manual drive...")
        except Exception as e:
            print(f"Error in manual drive loop: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        print("Cleaning up...")
        self.is_shutting_down = True
        # stop encoder printer
        self._enc_stop = True
        try:
            if self._enc_thread and self._enc_thread.is_alive():
                self._enc_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.set_motor_speeds(0, 0)
        except Exception:
            pass
        self.connection_manager.close()
        print("Cleanup complete")


def main():
    global GLOBAL_ROBOT
    robot = ManualDriveRobot()
    GLOBAL_ROBOT = robot

    if robot.setup():
        try:
            robot.run_pre_drive_tests()
            robot.run()
        finally:
            robot.cleanup()
    else:
        print("Failed to setup manual drive robot")


if __name__ == "__main__":
    main()
