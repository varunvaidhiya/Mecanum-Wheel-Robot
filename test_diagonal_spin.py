import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def calculate_checksum(data):
    return (sum(data) + 5) & 0xFF

def build_packet_diagonal(vx, vy, vz):
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    msg_type = 0x12 # FUNC_MOTION (Kinematics)
    CAR_TYPE = 1 # X3
    
    # Scale: m/s * 1000
    vx_int = int(vx * 1000)
    vy_int = int(vy * 1000)
    vz_int = int(vz * 1000)
    
    payload = struct.pack('<bhhh', CAR_TYPE, vx_int, vy_int, vz_int)
    
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

    print("=== DIAGONAL TEST (Wheel 2 ONLY) ===")
    print("Mecanum Math: If Vx = 0.2, Vy = 0.2")
    print("Wheel 1 Speed = (0.2 - 0.2) = 0.0 (Should NOT turn on)")
    print("Wheel 2 Speed = (0.2 + 0.2) = 0.4 (Should Spin!)")
    
    print("Ensure Motor is plugged into Port 2 (M2).")
    print("Sending Start Beep...")
    send_beep(ser)
    time.sleep(1.0)
    
    # Try multiple speeds
    speeds = [0.1, 0.2, 0.3]
    
    for s in speeds:
        vx = s
        vy = s
        print(f"Testing Speed: Vx={vx}, Vy={vy} (M1=0, M2={2*s})")
        pkt = build_packet_diagonal(vx, vy, 0.0) # W=0
        ser.write(bytearray(pkt))
        time.sleep(1.0) # Longer observation
        
    print("Stopping...")
    pkt_stop = build_packet_diagonal(0.0, 0.0, 0.0)
    ser.write(bytearray(pkt_stop))
    time.sleep(0.5)

    print("Sending End Beep (Wait for it)...")
    send_beep(ser)
    
    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
