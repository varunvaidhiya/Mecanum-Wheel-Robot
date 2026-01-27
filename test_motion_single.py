import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def calculate_checksum(data):
    return (sum(data) + 5) & 0xFF

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error: {e}")
        return

    print("Sending ONE Motion Packet (0.0, 0.0, 0.0)...")
    
    # Matches the logs: ['0xff', '0xfc', '0xa', '0x12', '0x1', '0x0', '0x0', '0x0', '0x0', '0x0', '0x0', '0x1d']
    # FF FC 0A 12 01 0000 0000 0000 CS
    
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    msg_type = 0x12
    
    # Payload: Type(1) + Vx(2) + Vy(2) + W(2)
    CAR_TYPE = 1
    vx = 0
    vy = 0
    w = 0
    payload = struct.pack('<bhhh', CAR_TYPE, vx, vy, w)
    
    packet = [HEAD, DEVICE_ID, 0, msg_type] + list(payload)
    packet[2] = len(packet) - 1
    
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    
    print(f"Packet: {[hex(x) for x in packet]}")
    ser.write(bytearray(packet))
    
    print("Sent. Waiting 2 seconds...")
    time.sleep(2)
    
    print("Sending Beep to check if alive...")
    # Beep packet
    payload_beep = struct.pack('<h', 100)
    packet_beep = [HEAD, DEVICE_ID, 0, 0x02] + list(payload_beep)
    packet_beep[2] = len(packet_beep) - 1
    packet_beep.append(calculate_checksum(packet_beep))
    
    ser.write(bytearray(packet_beep))
    print("Beep sent. Did it beep?")
    
    ser.close()

if __name__ == "__main__":
    main()
