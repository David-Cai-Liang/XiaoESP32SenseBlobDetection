import os
import struct
import sys
import time
from pynput import keyboard
import serial

# Enable VT100 ANSI escape sequence support on Windows terminal
if sys.platform == "win32":
    os.system("")

SERIAL_PORT = "COM9"  # Adjust for your OS ('/dev/ttyUSB0' or '/dev/ttyACM0')
BAUD_RATE = 115200

# Protocol Markers
TELEMETRY_HEADER = b"\x00\xAA\x55\xFF"
TELEMETRY_FOOTER = b"\xEE\xFF"
CONTROL_HEADER = b"\x00\xBB\x66\xFF"

PAYLOAD_SIZE = 24  # 4x uint16 (8B) + 4x float (16B)
TOTAL_FRAME_SIZE = 4 + PAYLOAD_SIZE + 2  # 30 Bytes Total Frame

PRINT_INTERVAL_S = 0.05  # Terminal refresh rate, decoupled from telemetry rate

# Keyboard state management
active_keys = set()


def on_press(key):
    try:
        if key.char:
            active_keys.add(key.char.lower())
    except AttributeError:
        pass


def on_release(key):
    try:
        if key.char:
            active_keys.discard(key.char.lower())
    except AttributeError:
        pass


def compute_motors():
    m1, m2, m3, m4 = 10, 0, 0, 0

    if "w" in active_keys:
        m2 += 50
        m3 += 50
    if "a" in active_keys:
        m3 += 50
    if "d" in active_keys:
        m2 += 50
    if "q" in active_keys:
        m4 += 50
    if "e" in active_keys:
        m1 += 50

    return [m1, m2, m3, m4]


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        print(f"Connected to Base Station on {SERIAL_PORT}")
    except Exception as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        sys.exit(1)

    # Clear OS buffer queue lag on launch
    ser.reset_input_buffer()

    # Start keyboard state listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("Control Active")
    print("Controls: Hold 'W' | 'A' | 'D' | 'Q' | 'E'")
    print("Press Ctrl+C to quit\n")

    buffer = bytearray()
    last_control_time = 0
    last_print_time = 0

    # Most recently parsed telemetry, printed on a fixed schedule (see below)
    # rather than once per frame, so a burst of telemetry never delays the
    # next control send.
    latest_vision = None  # (cx, cy, w, h)
    latest_imu = None     # (ax, ay, az, tz)

    try:
        while True:
            now = time.perf_counter()

            # 1. Transmit Motor Control Commands (~20 Hz)
            if now - last_control_time >= 0.05:
                last_control_time = now
                motors = compute_motors()
                payload = struct.pack("<4h", *motors)
                ser.write(CONTROL_HEADER + payload)

            # 2. Read all available Serial bytes directly into buffer
            if ser.in_waiting:
                buffer.extend(ser.read(ser.in_waiting))

            # 3. Parse ALL complete telemetry frames sequentially without discarding.
            #    No terminal I/O happens in here on purpose -- printing is throttled
            #    separately below so a backlog of frames can't stall the control loop.
            while len(buffer) >= TOTAL_FRAME_SIZE:
                idx = buffer.find(TELEMETRY_HEADER)
                if idx == -1:
                    buffer = buffer[-3:]
                    break
                if idx > 0:
                    del buffer[:idx]

                if len(buffer) >= TOTAL_FRAME_SIZE:
                    if buffer[4 + PAYLOAD_SIZE : TOTAL_FRAME_SIZE] == TELEMETRY_FOOTER:
                        raw_payload = buffer[4 : 4 + PAYLOAD_SIZE]
                        del buffer[:TOTAL_FRAME_SIZE]

                        cx, cy, w, h, ax, ay, az, tz = struct.unpack("<4H4f", raw_payload)
                        latest_vision = (cx, cy, w, h)
                        latest_imu = (ax, ay, az, tz)
                    else:
                        del buffer[:1]

            # 4. Print status at a fixed rate, independent of telemetry volume
            if latest_vision is not None and now - last_print_time >= PRINT_INTERVAL_S:
                last_print_time = now
                cx, cy, w, h = latest_vision
                ax, ay, az, tz = latest_imu
                curr_motors = compute_motors()

                sys.stdout.write(
                    f"\r\033[K[STATUS] Motors: {curr_motors} || "
                    f"Vision: CX:{cx:3d} CY:{cy:3d} W:{w:3d} H:{h:3d} || "
                    f"IMU: AX:{ax:5.1f} AY:{ay:5.1f} AZ:{az:5.1f}"
                )
                sys.stdout.flush()

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        motors = [0, 0, 0, 0]
        payload = struct.pack("<4h", *motors)
        for _ in range(5):
            ser.write(CONTROL_HEADER + payload)
            ser.flush()
            time.sleep(0.02)
        ser.close()
        listener.stop()


if __name__ == "__main__":
    main()
