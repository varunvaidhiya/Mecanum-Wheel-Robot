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
    msg_type = 0x10 # FUNC_MOTOR (Direct Control)
    
    # Payload: M1(2), M2(2), M3(2), M4(2)
    payload = struct.pack('<hhhh', int(m1), int(m2), int(m3), int(m4))
    
    packet = [HEAD, DEVICE_ID, 0, msg_type] + list(payload)
    packet[2] = len(packet) - 1 # Length calculation
    
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    return packet

def send_beep(ser):
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    msg_type = 0x02 # FUNC_BEEP
    
    payload = struct.pack('<h', 100) # 100ms
    packet = [HEAD, DEVICE_ID, 0, msg_type] + list(payload)
    packet[2] = len(packet) - 1
    
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    
    ser.write(bytearray(packet))
    print("Beep Sent.")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error: {e}")
        return

    print("=== DIRECT MOTOR CONTROL TEST (0x10) With Beeps ===")
    
    # 1. Beep Once BEFORE
    print("1. Beep Start...")
    send_beep(ser)
    time.sleep(1.0) # Wait for beep to finish

    print("Bypassing onboard kinematics.")
    print("Testing Motors at Speed 10 (Very Low)...")

    # 2. Drive forward
    t_end = time.time() + 2.0
    while time.time() < t_end:
        pkt = build_packet_motor(10, 10, 10, 10)
        ser.write(bytearray(pkt))
        time.sleep(0.1)
        
    print("Stopping...")
    pkt_stop = build_packet_motor(0, 0, 0, 0)
    ser.write(bytearray(pkt_stop))
    time.sleep(0.5)

    # 3. Beep Twice AFTER
    print("3. Beep End (Twice)...")
    send_beep(ser)
    time.sleep(0.2)
    send_beep(ser)
    
    ser.close()
    print("Done. Expected: Beep -> Move -> Beep Beep.")

if __name__ == "__main__":
    main()
