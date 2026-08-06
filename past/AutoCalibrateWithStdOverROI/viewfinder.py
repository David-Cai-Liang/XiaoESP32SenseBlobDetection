import serial
import cv2
import numpy as np
import struct

# Update this to match your Xiao COM port:
# Windows: 'COM3', 'COM4', etc.
# Mac/Linux: '/dev/ttyACM0' or '/dev/tty.usbmodem...'
SERIAL_PORT = 'COM8'
BAUD_RATE = 115200

MAGIC_HEADER = b'\xff\xaa\x55\xff'
HEADER_SIZE = 20  # 4 magic + 4 length + 12 thresholds (6 x int16)

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Xiao ESP32S3 on {SERIAL_PORT}.")
        print("Press 'q' on the image window to exit.\n")
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    buffer = bytearray()
    last_print_time = 0

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

            # Unpack binary frame header
            _, payload_len, l_min, l_max, a_min, a_max, b_min, b_max = struct.unpack(
                '<4sIhhhhhh', buffer[:HEADER_SIZE]
            )

            if len(buffer) < HEADER_SIZE + payload_len:
                break  # Wait for complete JPEG payload

            jpg_data = buffer[HEADER_SIZE : HEADER_SIZE + payload_len]
            buffer = buffer[HEADER_SIZE + payload_len:]

            # Decode JPEG image
            np_arr = np.frombuffer(jpg_data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is not None:
                # Print live C++ struct string in terminal once per second
                import time
                if time.time() - last_print_time >= 1.0:
                    last_print_time = time.time()
                    print(f"{l_min}, {l_max}, {a_min}, {a_max}, {b_min}, {b_max}")

                # Overlay live LAB parameters on video feed
                text = f"L:[{l_min},{l_max}] A:[{a_min},{a_max}] B:[{b_min},{b_max}]"
                cv2.putText(img, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                cv2.imshow("Xiao USB LAB Calibrator (Low Latency)", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    ser.close()
                    cv2.destroyAllWindows()
                    return

if __name__ == '__main__':
    main()
