import struct
import sys
import time
import os
import serial

# Enable VT100 ANSI escape sequence support on Windows terminal
if sys.platform == "win32":
    os.system("")

SERIAL_PORT = "COM9"  # Adjust for your OS ('/dev/ttyUSB0' or '/dev/ttyACM0')
BAUD_RATE = 115200    # Set to 921600 if updated in base_station.ino

TELEMETRY_HEADER = b"\x00\xAA\x55\xFF"
TELEMETRY_FOOTER = b"\xEE\xFF"
PAYLOAD_SIZE = 24  # 4x uint16 (8B) + 4x float (16B)
TOTAL_FRAME_SIZE = 4 + PAYLOAD_SIZE + 2  # 30 Bytes Total Frame


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        print(f"Connected to Base Station on {SERIAL_PORT}")
    except Exception as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        sys.exit(1)

    # Flush stale queued packets from OS buffer on launch
    ser.reset_input_buffer()

    print("Telemetry Latency & Frame Rate Benchmark Active")
    print("Measuring inter-arrival latency, jitter, and FPS... (Ctrl+C to stop)\n\n")

    buffer = bytearray()

    # Latency & Throughput tracking variables
    last_frame_time = None
    delta_ms = 0.0
    frame_deltas = []
    max_history = 200  # Rolling window size for averaging
    total_frames = 0
    start_bench_time = time.perf_counter()

    try:
        while True:
            # Immediately skip stale backlog if OS buffer builds up
            if ser.in_waiting > TOTAL_FRAME_SIZE * 5:
                raw = ser.read(ser.in_waiting)
                buffer.extend(raw)
                last_idx = buffer.rfind(TELEMETRY_HEADER)
                if last_idx != -1 and len(buffer) - last_idx >= TOTAL_FRAME_SIZE:
                    buffer = buffer[last_idx:]
            elif ser.in_waiting:
                buffer.extend(ser.read(ser.in_waiting))

            # Parse and align frame
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

                        now = time.perf_counter()

                        # 1. Calculate Inter-Frame Delta Latency
                        if last_frame_time is not None:
                            delta_ms = (now - last_frame_time) * 1000.0
                            frame_deltas.append(delta_ms)
                            if len(frame_deltas) > max_history:
                                frame_deltas.pop(0)

                        last_frame_time = now
                        total_frames += 1

                        # 2. Compute Benchmark Statistics
                        avg_dt = sum(frame_deltas) / len(frame_deltas) if frame_deltas else 0.0
                        min_dt = min(frame_deltas) if frame_deltas else 0.0
                        max_dt = max(frame_deltas) if frame_deltas else 0.0
                        fps = 1000.0 / avg_dt if avg_dt > 0 else 0.0

                        # 3. Unpack Full Telemetry Packet
                        cx, cy, w, h, ax, ay, az, tz = struct.unpack("<4H4f", raw_payload)

                        # 4. Print Telemetry (Line 1) and Latency Metrics (Line 2)
                        sys.stdout.write(
                            f"\r\033[K[TELEMETRY] Vision: CX:{cx:3d} CY:{cy:3d} W:{w:3d} H:{h:3d}  ||  "
                            f"IMU: AX:{ax:6.2f} AY:{ay:6.2f} AZ:{az:6.2f} TZ:{tz:6.2f}\n"
                            f"\r\033[K[LATENCY]   Delta: {delta_ms:5.1f}ms | Avg: {avg_dt:5.1f}ms | "
                            f"Min: {min_dt:5.1f}ms | Max: {max_dt:5.1f}ms | "
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


if __name__ == "__main__":
    main()
