import struct
import serial

# --- Configuration ---
SERIAL_PORT = 'COM8'
BAUD_RATE = 115200

MAGIC_BYTES = b'\xFF\xAA\x55\xFF'

# Payload: < (little-endian), I (uint32 timestamp ms), 11H (11 x uint16 telemetry fields)
PAYLOAD_FORMAT = "<I11H"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)  # 26 bytes
FRAME_SIZE = len(MAGIC_BYTES) + PAYLOAD_SIZE    # 30 bytes total

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print(f"Connected to {SERIAL_PORT}. Listening for telemetry frames...")

    buffer = bytearray()

    while True:
        # Read available bytes into buffer
        buffer += ser.read(ser.in_waiting or 1)

        while len(buffer) >= FRAME_SIZE:
            # Find location of magic header sequence
            idx = buffer.find(MAGIC_BYTES)

            if idx == -1:
                # Magic not found: keep last 3 bytes (in case header was cut mid-read)
                buffer = buffer[-(len(MAGIC_BYTES) - 1):]
                break

            if idx > 0:
                # Discard misaligned bytes preceding header
                print(f"Misaligned stream: discarded {idx} byte(s)")
                buffer = buffer[idx:]

            # Verify complete frame is present
            if len(buffer) < FRAME_SIZE:
                break

            # Extract payload bytes and slice consumed frame from buffer
            payload = buffer[len(MAGIC_BYTES):FRAME_SIZE]
            buffer = buffer[FRAME_SIZE:]

            # Unpack payload
            unpacked = struct.unpack(PAYLOAD_FORMAT, payload)
            timestamp_ms = unpacked[0]
            (
                hb, roi_x, roi_y, roi_w, roi_h,
                cx, cy, w, h, max_w, max_h
            ) = unpacked[1:]

            print(f"Time: {timestamp_ms:<5} | Target CX: {cx}, CY: {cy}")

if __name__ == "__main__":
    main()
