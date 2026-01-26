
import serial
import time
import struct

def calculate_checksum(data):
    # Sum of all bytes in data
    checksum = sum(data) & 0xFF
    return checksum

def create_packet(msg_type, payload):
    # Header: FF FB
    # Length: 1 (Len) + 1 (Type) + Len(Payload) + 1 (Checksum)
    length = 1 + 1 + len(payload) + 1
    
    # Packet body: Len, Type, Payload
    body = [length, msg_type] + list(payload)
    
    # Calculate checksum
    checksum = calculate_checksum(body)
    
    # Full packet
    packet = [0xFF, 0xFB] + body + [checksum]
    return bytearray(packet)

def send_packet(ser, msg_type, payload, desc):
    packet = create_packet(msg_type, payload)
    print(f"Testing {desc}: Type={hex(msg_type)} Payload={list(payload)}")
    print(f"Sending: {[hex(b) for b in packet]}")
    ser.write(packet)
    time.sleep(1.0)
    # Stop
    stop_packet = create_packet(msg_type, [0]*len(payload))
    ser.write(stop_packet)
    time.sleep(0.5)

def main():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        print("Opened serial port.")
        
        # Test types 0x01 to 0x05
        # Assume payload for motors might be 4 bytes (4 signed chars) or 8 bytes (4 shorts) or 3 shorts (vx, vy, vz)
        
        # Test 1: Type 0x01, 3 velocities (vx, vy, vz) as shorts (multiplied by 100 or 1000)
        # 100 -> 0x64
        vx = 200
        vy = 0
        vz = 0
        payload_3short = struct.pack('<hhh', vx, vy, vz)
        send_packet(ser, 0x01, payload_3short, "Type 0x01 (Vel x,y,z shorts)")
        
        # Test 2: Type 0x01, 4 wheel speeds as signed chars
        payload_4char = struct.pack('<bbbb', 30, 30, 30, 30)
        send_packet(ser, 0x01, payload_4char, "Type 0x01 (4 wheel chars)")

        # Test 3: Type 0x02, same payloads
        send_packet(ser, 0x02, payload_3short, "Type 0x02 (Vel x,y,z shorts)")
        send_packet(ser, 0x02, payload_4char, "Type 0x02 (4 wheel chars)")
        
        ser.close()
        print("Done.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
