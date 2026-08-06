import struct
import serial

# --- Configuration ---
SERIAL_PORT = 'COM8'
BAUD_RATE = 115200

# Bytes {0xFF, 0xAA, 0x55, 0xFF} interpreted as little-endian uint32 (<I)
MAGIC_HEADER = 0xFF55AAFF

# Format: < (little-endian), I (uint32 timestamp ms), 11H (11 x uint16 telemetry fields)
PAYLOAD_FORMAT = "<I11H"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)  # 26 bytes

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print(f"Connected to {SERIAL_PORT}. Listening for telemetry frames...")

    while True:
        # Step 1: Read 4 header bytes and verify sync marker
        header_bytes = ser.read(4)
        if len(header_bytes) < 4:
            continue

        magic = struct.unpack("<I", header_bytes)[0]

        if magic == MAGIC_HEADER:
            # Step 2: Read the remaining 26 payload bytes
            payload = ser.read(PAYLOAD_SIZE)
            if len(payload) == PAYLOAD_SIZE:
                # Step 3: Unpack timestamp (ms) + 11 telemetry fields
                unpacked = struct.unpack(PAYLOAD_FORMAT, payload)

                timestamp_ms = int(unpacked[0])

                (
                    hb, roi_x, roi_y, roi_w, roi_h,
                    cx, cy, w, h, max_w, max_h
                ) = unpacked[1:]

                print(f"Time: {timestamp_ms:<5} | Target CX: {cx}, CY: {cy}")
        else:
            # Step 4: Resynchronize stream byte-by-byte if misaligned
            print("Missaligned")
            ser.read(1)

if __name__ == "__main__":
    main()
