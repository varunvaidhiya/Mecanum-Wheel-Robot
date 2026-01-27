import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def calculate_checksum(data):
    return (sum(data) + 5) & 0xFF

def build_packet(vx, vy, w):
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    msg_type = 0x12
    CAR_TYPE = 1
    
    payload = struct.pack('<bhhh', CAR_TYPE, int(vx*1000), int(vy*1000), int(w*1000))
    packet = [HEAD, DEVICE_ID, 0, msg_type] + list(payload)
    packet[2] = len(packet) - 1
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    return packet

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error: {e}")
        return

    print("=== STRESS TEST ===")
    print("Sending 10Hz Motor Commands (Speed 0.2) for 3 seconds...")
    print("Watch the robot!")

    # 1. Move Forward for 3 seconds
    t_end = time.time() + 3.0
    while time.time() < t_end:
        pkt = build_packet(0.2, 0.0, 0.0)
        ser.write(bytearray(pkt))
        time.sleep(0.1) # 10 Hz
    
    # 2. Stop
    print("Stopping...")
    pkt = build_packet(0.0, 0.0, 0.0)
    ser.write(bytearray(pkt))
    time.sleep(0.5)
    
    # 3. Check survival
    print("Sending Beep to check if alive...")
    payload_beep = struct.pack('<h', 100)
    packet_beep = [0xFF, 0xFC, 0, 0x02] + list(payload_beep)
    packet_beep[2] = len(packet_beep) - 1
    packet_beep.append(calculate_checksum(packet_beep))
    
    ser.write(bytearray(packet_beep))
    
    ser.close()
    print("Done. Did it move? Did it beep at the end?")

if __name__ == "__main__":
    main()
