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

    print("=== CRAWL TEST (Write Only) ===")
    print("Ramping speed from 0.00 to 0.10 very slowly...")
    
    current_vx = 0.0
    
    # Ramp up for 5 seconds
    # 0.00 to 0.10 in 50 steps -> 0.002 per step
    for i in range(50):
        current_vx += 0.002
        print(f"Speed: {current_vx:.3f}")
        
        pkt = build_packet(current_vx, 0.0, 0.0)
        ser.write(bytearray(pkt))
        time.sleep(0.1)
        
    print("Holding 0.10 for 2 seconds...")
    for i in range(20):
        ser.write(bytearray(pkt))
        time.sleep(0.1)

    print("Stopping...")
    pkt = build_packet(0.0, 0.0, 0.0)
    ser.write(bytearray(pkt))
    time.sleep(0.5)
    
    # Check survival
    print("Sending Beep...")
    payload_beep = struct.pack('<h', 100)
    packet_beep = [0xFF, 0xFC, 0, 0x02] + list(payload_beep)
    packet_beep[2] = len(packet_beep) - 1
    packet_beep.append(calculate_checksum(packet_beep))
    
    ser.write(bytearray(packet_beep))
    
    ser.close()
    print("Done. Did it move? Did it beep?")

if __name__ == "__main__":
    main()
