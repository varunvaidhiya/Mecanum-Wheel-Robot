import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def calculate_checksum(data):
    return (sum(data) + 5) & 0xFF

def build_packet_motor(m1, m2, m3, m4):
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    msg_type = 0x10 # FUNC_MOTOR
    
    # Payload: M1(2), M2(2), M3(2), M4(2)
    payload = struct.pack('<hhhh', int(m1), int(m2), int(m3), int(m4))
    
    packet = [HEAD, DEVICE_ID, 0, msg_type] + list(payload)
    packet[2] = len(packet) - 1
    
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    return packet

def send_beep(ser):
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    msg_type = 0x02
    
    payload = struct.pack('<h', 100)
    packet = [HEAD, DEVICE_ID, 0, msg_type] + list(payload)
    packet[2] = len(packet) - 1
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    ser.write(bytearray(packet))

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error: {e}")
        return

    print("=== RAMP TEST: PORT 2 ONLY ===")
    print("Ensure Motor is in Port 2 (M2).")
    print("Sending Beep...")
    send_beep(ser)
    time.sleep(1.0)
    
    speeds = [100, 200, 300, 400, 500, 600, 700, 800, 900]
    
    for s in speeds:
        print(f"Testing Speed: {s} / 1000")
        pkt = build_packet_motor(0, s, 0, 0) # Only M2
        ser.write(bytearray(pkt))
        time.sleep(1.0) # Longer duration to observe movement
        
    print("Stopping...")
    pkt_stop = build_packet_motor(0, 0, 0, 0)
    ser.write(bytearray(pkt_stop))
    time.sleep(0.5)

    print("Sending End Beep...")
    send_beep(ser)
    
    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
