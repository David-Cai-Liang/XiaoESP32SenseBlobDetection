import serial
import cv2
import numpy as np
import struct
import datetime

# Update to match your Xiao COM port:
# Windows: 'COM3', 'COM4', etc. | Mac/Linux: '/dev/ttyACM0'
SERIAL_PORT = 'COM29'
BAUD_RATE = 115200

MAGIC_HEADER = b'\xff\xaa\x55\xff'
# Header calculation: 4 (magic) + 4 (payload_len) + 10 (5x uint16) + 16 (4x float32) = 34 bytes
HEADER_SIZE = 34
UNPACK_FORMAT = '<4sI5H4f'

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Xiao ESP32S3 on {SERIAL_PORT}.")
        print("Press 'q' on the video window to exit.\n")
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    buffer = bytearray()

    while True:
        data = ser.read(ser.in_waiting or 1)
        if data:
            buffer += data

        while len(buffer) >= HEADER_SIZE:
            idx = buffer.find(MAGIC_HEADER)
            if idx == -1:
                buffer.clear()
                break
            if idx > 0:
                buffer = buffer[idx:]

            if len(buffer) < HEADER_SIZE:
                break

            # Unpack binary frame header & telemetry structure
            unpacked = struct.unpack(UNPACK_FORMAT, buffer[:HEADER_SIZE])
            payload_len = unpacked[1]

            # Extract packed telemetry fields: hb, cx, cy, w, h, ax, ay, az, tz
            hb, cx, cy, w, h, ax, ay, az, tz = unpacked[2:]
            telem_data = {
                "hb": hb,
                "roi": (cx, cy, w, h),
                "accel": (round(ax, 3), round(ay, 3), round(az, 3)),
                "rot_z": round(tz, 3)
            }

            if len(buffer) < HEADER_SIZE + payload_len:
                break  # Wait for complete JPEG payload

            if payload_len == 0:
                print(f"{datetime.datetime.now()}: {telem_data}")
                buffer = buffer[HEADER_SIZE:]
                continue

            jpg_data = buffer[HEADER_SIZE : HEADER_SIZE + payload_len]
            buffer = buffer[HEADER_SIZE + payload_len:]

            # Decode JPEG image
            np_arr = np.frombuffer(jpg_data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is not None:
                print(f"{datetime.datetime.now()}: {telem_data}")

                # Display frame with real-time video stream
                cv2.imshow("Balloon Race USB Stream", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    ser.close()
                    cv2.destroyAllWindows()
                    return

if __name__ == '__main__':
    main()
