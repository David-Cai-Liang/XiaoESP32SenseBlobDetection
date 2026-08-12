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
    m1, m2, m3, m4 = 0, 0, 0, 0

    if "w" in active_keys:
        m1 += 25
        m2 += 25
    if "a" in active_keys:
        m1 += 25
    if "d" in active_keys:
        m2 += 25
    if "q" in active_keys:
        m3 += 25
    if "e" in active_keys:
        m4 += 25

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

    print("Control & Benchmark Active")
    print("Controls: Hold 'W' (M1+M2=25) | 'A' (+M1=25) | 'D' (+M2=25) | 'Q' (M3=25) | 'E' (M4=25)")
    print("Press Ctrl+C to quit\n\n")

    buffer = bytearray()
    last_control_time = 0

    # Latency tracking variables
    last_frame_time = None
    delta_ms = 0.0
    frame_deltas = []
    max_history = 200  # Rolling window size for averaging
    total_frames = 0
    start_bench_time = time.perf_counter()

    try:
        while True:
            # 1. Transmit Motor Control Commands (~20 Hz)
            now = time.perf_counter()
            if now - last_control_time >= 0.05:
                last_control_time = now
                motors = compute_motors()
                payload = struct.pack("<4h", *motors)
                ser.write(CONTROL_HEADER + payload)

            # 2. Prevent Buffer Backlog Lag (Discard Stale Packets)
            if ser.in_waiting > TOTAL_FRAME_SIZE * 5:
                raw = ser.read(ser.in_waiting)
                buffer.extend(raw)
                last_idx = buffer.rfind(TELEMETRY_HEADER)
                if last_idx != -1 and len(buffer) - last_idx >= TOTAL_FRAME_SIZE:
                    buffer = buffer[last_idx:]
            elif ser.in_waiting:
                buffer.extend(ser.read(ser.in_waiting))

            # 3. Parse & Benchmark Incoming Telemetry
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

                        recv_time = time.perf_counter()

                        if last_frame_time is not None:
                            total_frames += 1
                            delta_ms = (recv_time - last_frame_time) * 1000.0
                            frame_deltas.append(delta_ms)
                            if len(frame_deltas) > max_history:
                                frame_deltas.pop(0)

                        last_frame_time = recv_time

                        avg_dt = sum(frame_deltas) / len(frame_deltas) if frame_deltas else 0.0
                        fps = 1000.0 / avg_dt if avg_dt > 0 else 0.0

                        cx, cy, w, h, ax, ay, az, tz = struct.unpack("<4H4f", raw_payload)
                        curr_motors = compute_motors()

                        # Terminal display: Line 1 (Telemetry & Motors) | Line 2 (Latency Metrics)
                        sys.stdout.write(
                            f"\r\033[K[STATUS] Motors: {curr_motors} || "
                            f"Vision: CX:{cx:3d} CY:{cy:3d} W:{w:3d} H:{h:3d} || "
                            f"IMU: AX:{ax:5.1f} AY:{ay:5.1f} AZ:{az:5.1f}\n"
                            f"\r\033[K[LATENCY] Delta: {delta_ms:5.1f}ms | Avg: {avg_dt:5.1f}ms | "
                            f"Rate: {fps:4.1f} FPS | Queue: {ser.in_waiting}B\033[A"
                        )
                        sys.stdout.flush()
                    else:
                        del buffer[:1]

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n\n\n--- Benchmark Summary ---")
        if frame_deltas:
            total_time = time.perf_counter() - start_bench_time
            avg_ms = sum(frame_deltas) / len(frame_deltas)
            print(f"Total Frames Received : {total_frames}")
            print(f"Total Test Duration   : {total_time:.2f} s")
            print(f"Average Frame Delta   : {avg_ms:.2f} ms")
            print(f"Min / Max Delta Jitter: {min(frame_deltas):.2f} ms / {max(frame_deltas):.2f} ms")
            print(f"Average Throughput    : {total_frames / total_time:.2f} FPS")
        print("Exiting...")
    finally:
        ser.close()
        listener.stop()


if __name__ == "__main__":
    main()
