import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def chk(data):
    return sum(data) & 0xFF

def build_packet(func_id, payload):
    # Length = 2 + len(payload) (Standard Yahboom)
    # based on prev analysis where Length included Type+Payload+Check?
    # No, let's stick to the one that matches the sniffer logs:
    # Len byte value = 1 (Type) + Len(Payload) + 1 (Check) + 1 (Len)?
    # Sniff: FF FB 09 0C ... 
    # 09 = 1(Len) + 1(Type) + 6(Payload) + 1(Check).
    # Len Value = 2 + len(payload) + 1
    
    length_val = 3 + len(payload)
    body = [length_val, func_id] + list(payload)
    checksum = chk(body)
    return [0xFF, 0xFB] + body + [checksum]

def run(ser, fid, payload, desc):
    pkt = build_packet(fid, payload)
    print(f"Testing ID 0x{fid:02X} - {desc}")
    print(f"  Sent: {[hex(x) for x in pkt]}")
    ser.write(bytearray(pkt))
    time.sleep(0.5)

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(e)
        return

    print("Scanning for BEEPS or LED Changes...")
    print("Press ENTER to send each command.")
    
    # Test Buzzer? (Usually single byte: 1=on, 0=off)
    # Try IDs 0x03, 0x05, 0x06 with 1 byte payload
    for fid in [0x03, 0x04, 0x05, 0x06]:
        input(f"Try ID 0x{fid:02X} (Payload: 0x01 = ON)?")
        run(ser, fid, [0x01], "Single Byte ON")
        time.sleep(1)
        run(ser, fid, [0x00], "Single Byte OFF")

    # Test LED? (R, G, B, Mode?) -> 4 bytes?
    # Try IDs 0x04, 0x08 with 4 bytes
    for fid in [0x04, 0x08]:
        input(f"Try ID 0x{fid:02X} (Payload: 255, 0, 0, 0 = RED)?")
        run(ser, fid, [255, 0, 0, 0], "Red Color?")
        time.sleep(1)
        run(ser, fid, [0, 255, 0, 0], "Green Color?")
        time.sleep(1)
        run(ser, fid, [0, 0, 0, 0], "OFF")

    ser.close()

if __name__ == "__main__":
    main()
