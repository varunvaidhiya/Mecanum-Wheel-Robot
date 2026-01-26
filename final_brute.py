
import serial
import time
import struct

def chk(data):
    return sum(data) & 0xFF

def send(ser, msg_type, payload_fmt, values, desc):
    try:
        payload = struct.pack(payload_fmt, *values)
    except:
        return # Skip invalid packs

    # Length = Type(1) + Payload(N) + Check(1) + Len(1) = N + 3
    length = 3 + len(payload)
    body = [length, msg_type] + list(payload)
    checksum = chk(body)
    packet = [0xFF, 0xFB] + body + [checksum]
    
    print(f"Trying {desc} ... (Type {hex(msg_type)}, Len {length})")
    # Send burst
    for _ in range(5):
        ser.write(bytearray(packet))
        time.sleep(0.01)
    time.sleep(1.0) # Wait for movement

def main():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    except:
        print("Error opening port. Stop ROS first.")
        return

    # Wakeup
    ser.write(b"INIT\n")
    ser.write(b"MODE_MECANUM\n")
    time.sleep(1)

    print("=== STARTING BRUTE FORCE ===")
    
    # 1. Type 0x01 (Car Run) - 3 Shorts (Vx, Vy, Vz)
    # Speed 300 (likely mm/s)
    send(ser, 0x01, '<hhh', (300, 0, 0), "Type 0x01 (Car): 3 Shorts (300)")

    # 2. Type 0x01 (Car Run) - 3 Floats
    # Speed 0.3
    send(ser, 0x01, '<fff', (0.3, 0.0, 0.0), "Type 0x01 (Car): 3 Floats (0.3)")

    # 3. Type 0x02 (Motor) - 4 Bytes (PWM)
    # Speed 50
    send(ser, 0x02, '<bbbb', (50, 50, 50, 50), "Type 0x02 (Motor): 4 Bytes (50)")

    # 4. Type 0x02 (Motor) - 4 Shorts (Speed)
    # Speed 300
    send(ser, 0x02, '<hhhh', (300, 300, 300, 300), "Type 0x02 (Motor): 4 Shorts (300)")

    # 5. Type 0x04 (Control?) - 4 Bytes
    send(ser, 0x04, '<bbbb', (50, 50, 50, 50), "Type 0x04? : 4 Bytes (50)")

    print("=== DONE ===")
    ser.close()

if __name__ == "__main__":
    main()
