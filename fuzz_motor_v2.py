
import serial
import time
import struct

def calculate_checksum(data):
    return sum(data) & 0xFF

def create_packet(msg_type, payload):
    # Header: FF FB
    # Length: 1 (Len) + 1 (Type) + Len(Payload) + 1 (Checksum)
    length = 1 + 1 + len(payload) + 1
    body = [length, msg_type] + list(payload)
    checksum = calculate_checksum(body)
    packet = [0xFF, 0xFB] + body + [checksum]
    return bytearray(packet)

def send_stream(ser, msg_type, payload, duration=1.5):
    packet = create_packet(msg_type, payload)
    print(f"Testing Type={hex(msg_type)} PayloadLen={len(payload)}. Sending for {duration}s...")
    print(f"Packet: {[hex(b) for b in packet]}")
    
    start = time.time()
    while time.time() - start < duration:
        ser.write(packet)
        time.sleep(0.05) # 20Hz

    # Stop packet (zeros)
    stop_payload = list(payload)
    for i in range(len(stop_payload)):
        stop_payload[i] = 0
    stop_packet = create_packet(msg_type, stop_payload)
    ser.write(stop_packet)
    time.sleep(0.2)
    ser.flushInput() # Clear buffer of response junk

def main():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        print("Opened serial port.")
        
        # Test Case A: Type 0x01, 3 shorts (Vx, Vy, Vz) - Most likely for Mecanum
        # Value 300 seems reasonable
        speed = 300
        payload_3short = struct.pack('<hhh', speed, 0, 0)
        send_stream(ser, 0x01, payload_3short)
        
        # Test Case B: Type 0x01, 4 chars (M1, M2, M3, M4)
        payload_4char = struct.pack('<bbbb', 50, 50, 50, 50)
        send_stream(ser, 0x01, payload_4char)

        # Test Case C: Type 0x02, 3 shorts
        send_stream(ser, 0x02, payload_3short)

        # Test Case D: Type 0x03, 3 shorts
        send_stream(ser, 0x03, payload_3short)

        # Test Case E: Type 0x04, 3 shorts (Some Yahboom boards use 0x04 for Hex move)
        send_stream(ser, 0x04, payload_3short)
        
        ser.close()
        print("Done.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
