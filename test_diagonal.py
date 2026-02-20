import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def calculate_checksum(data):
    return (sum(data) + 5) & 0xFF

def build_packet_kinematics(vx, vy, w):
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    msg_type = 0x12 # FUNC_MOTION (Kinematics)
    CAR_TYPE = 1 # X3
    
    # Scale: m/s * 1000
    vx_int = int(vx * 1000)
    vy_int = int(vy * 1000)
    w_int  = int(w * 1000)
    
    payload = struct.pack('<bhhh', CAR_TYPE, vx_int, vy_int, w_int)
    
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

    print("=== DIAGONAL TEST (Workaround) ===")
    print("Trying to Move Wheel 2 without Moving Wheel 1 (using Kinematics).")
    print("Sending Beep...")
    send_beep(ser)
    time.sleep(1.0)
    
    # Mecanum Math:
    # V1 = Vx - Vy - k*w
    # If Vx = 0.1, Vy = 0.1, W = 0
    # V1 = 0.1 - 0.1 = 0.0 (Should not crash!)
    # V2 = 0.1 + 0.1 = 0.2 (Should move!)
    
    print("Moving Diagonally (Vx=0.1, Vy=0.1)...")
    pkt = build_packet_kinematics(0.1, 0.1, 0.0)
    
    t_end = time.time() + 2.0
    while time.time() < t_end:
        ser.write(bytearray(pkt))
        time.sleep(0.1)
        
    print("Stopping...")
    pkt_stop = build_packet_kinematics(0.0, 0.0, 0.0)
    ser.write(bytearray(pkt_stop))
    time.sleep(0.5)

    print("Sending End Beep...")
    send_beep(ser)
    
    ser.close()
    print("Done. Did it move? Did it freeze?")

if __name__ == "__main__":
    main()
