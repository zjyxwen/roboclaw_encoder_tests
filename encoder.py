from time import sleep
import sys

from roboclaw import Roboclaw

# pure encoder readings, no autonomous drive
# use this file when you want to purely test the encoder readings when manually moving the motors

def try_open_roboclaw(port, baudrate, address):
    rc = Roboclaw(port, baudrate)
    if not rc.Open():
        return None
    try:
        ok, _version = rc.ReadVersion(address)
        if ok:
            return rc
    except Exception:
        pass
    try:
        rc._port.close()
    except Exception:
        pass
    return None


def get_roboclaw(address=0x80, baudrate=38400):
    # try common ports to open roboclaw-- should be one of the tty ones
    candidate_ports = [
        "/dev/ttyACM0",
        "/dev/ttyUSB0",
        "/dev/ttyS0",
        "COM3",
        "COM4",
        "COM5",
    ]

    for port in candidate_ports:
        try:
            rc = try_open_roboclaw(port, baudrate, address)
            if rc is not None:
                print(f"Connected to RoboClaw on {port} @ {baudrate} baud")
                return rc, port
        except Exception:
            pass

    # fallback to default port
    port = "/dev/ttyS0"
    rc = try_open_roboclaw(port, baudrate, address)
    if rc is not None:
        print(f"Connected to RoboClaw on {port} @ {baudrate} baud")
        return rc, port

    return None, None


def print_encoder_values(roboclaw, address):
    # printing encoder values :D thank you packet_serial.py
    enc1 = roboclaw.ReadEncM1(address)
    enc2 = roboclaw.ReadEncM2(address)

    if enc1[0]:
        m1_text = f"M1 Ticks: {enc1[1]}, Status: {enc1[2]}"
    else:
        m1_text = "M1: read failed"

    if enc2[0]:
        m2_text = f"M2 Ticks: {enc2[1]}, Status: {enc2[2]}"
    else:
        m2_text = "M2: read failed"

    print(f"Encoders -> {m1_text} | {m2_text}")


def main():
    address = 0x80

    roboclaw, port = get_roboclaw(address=address)
    if roboclaw is None:
        print("failed to open roboclaw connection on ports tried :(")
        sys.exit(1)

    try:
        print("starting  encoder read (press ctrl+c to stop)")
        while True:
            print_encoder_values(roboclaw, address)
            sleep(0.25)
    except KeyboardInterrupt:
        print("\nstopping...")
    finally:
        try:
            roboclaw._port.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
