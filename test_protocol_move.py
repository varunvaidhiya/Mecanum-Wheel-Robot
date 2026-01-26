import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def calculate_checksum(data):
    # Sum of all bytes in data
    return sum(data) & 0xFF

def send_packet(ser, func_id, speed_x, speed_y, speed_z):
    # Construct Payload
    # Structure Hypothesis:
    # Header: FF FB
    # Length: 1 (Len) + 1 (Func) + 6 (Data) + 1 (CS) = 9 (0x09)
    # Func: 0x01
    # Data: X (2), Y (2), Z (2)
    
    # Try Little Endian first
    # 300 -> 0x012C -> 2C 01
    
    payload_data = struct.pack('<hhh', int(speed_x), int(speed_y), int(speed_z))
    # payload_data length is 6
    
    # Message Body (Length + Func + Data)
    # Length byte value = 1 + 1 + 6 + 1 = 9
    length_byte = 9
    
    msg_body = [length_byte, func_id] + list(payload_data)
    
    checksum = calculate_checksum(msg_body)
    packet = [0xFF, 0xFB] + msg_body + [checksum]
    
    print(f"Sending: {[hex(b) for b in packet]}")
    ser.write(bytearray(packet))

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error: {e}")
        return

    print("Test 1: Move X (Forward) speed 20")
    send_packet(ser, 0x01, 20, 0, 0) # Slow speed
    time.sleep(1)
    send_packet(ser, 0x01, 0, 0, 0) # Stop
    time.sleep(1)

    print("Test 2: Move X (Forward) speed 100")
    send_packet(ser, 0x01, 100, 0, 0)
    time.sleep(1)
    send_packet(ser, 0x01, 0, 0, 0) # Stop
    time.sleep(1)

    print("Test 3: Move Y (Sideways) speed 100")
    send_packet(ser, 0x01, 0, 100, 0)
    time.sleep(1)
    send_packet(ser, 0x01, 0, 0, 0) # Stop
    
    print("Test 4: Rotation speed 100")
    send_packet(ser, 0x01, 0, 0, 100)
    time.sleep(1)
    send_packet(ser, 0x01, 0, 0, 0) # Stop

    print("Test 5: Try Function 0x02 (Servo/Motor?) with 4 bytes?")
    # Maybe 0x01 is car, 0x02 is something else?
    # Let's stick to 0x01 for now.
    
    ser.close()

if __name__ == "__main__":
    main()
