import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def calculate_checksum(data):
    return (sum(data) + 5) & 0xFF

def build_packet_kinematics(car_type, vx, vy, w):
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    msg_type = 0x12 # FUNC_MOTION
    
    vx_int = int(vx * 1000)
    vy_int = int(vy * 1000)
    w_int  = int(w * 1000)
    
    payload = struct.pack('<bhhh', car_type, vx_int, vy_int, w_int)
    
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

    print("=== CAR TYPE SCAN: PORT 2 ONLY ===")
    print("Trying different CAR_TYPE IDs...")
    print("Ensure Motor is plugged into Port 2 (M2).")
    
    # Send Start Beep
    send_beep(ser)
    time.sleep(1.0)
    
    # Try Types 1 to 5
    # Move Diagonally (Vx=0.2, Vy=0.2) to keep M1=0
    
    vx = 0.2
    vy = 0.2 
    
    for c_type in range(1, 6):
        print(f"Testing CAR_TYPE: {c_type}...")
        
        # Drive for 2 seconds
        t_end = time.time() + 2.0
        while time.time() < t_end:
            pkt = build_packet_kinematics(car_type=c_type, vx=vx, vy=vy, w=0)
            ser.write(bytearray(pkt))
            time.sleep(0.1)
        
        # Stop
        pkt_stop = build_packet_kinematics(car_type=c_type, vx=0, vy=0, w=0)
        ser.write(bytearray(pkt_stop))
        time.sleep(1.0)
        
    print("Stopping...")
    print("Sending End Beep...")
    send_beep(ser)
    
    ser.close()
    print("Done. Did any Type move the wheel?")

if __name__ == "__main__":
    main()
