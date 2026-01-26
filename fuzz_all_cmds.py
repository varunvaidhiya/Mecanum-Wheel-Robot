import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def list_checksums(data):
    s = sum(data) & 0xFF
    return [s]

def build_packet(func_id, payload):
    # Confirmed Protocol:
    # Header: FF FB
    # Length Byte: 3 + len(payload) (Counts L, T, and CS + Data)
    # Checksum: Sum(Length + Type + Payload)
    
    length_val = 3 + len(payload)
    body = [length_val, func_id] + list(payload)
    
    checksum = sum(body) & 0xFF
    return [0xFF, 0xFB] + body + [checksum]

def run_test(ser, fid, payload):
    pkt = build_packet(fid, payload)
    ser.write(bytearray(pkt))
    time.sleep(0.1) 

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(e)
        return

    print("=== FULL COMMAND FUZZER ===")
    print("Press CTRL+C if the robot BEEPS or MOVES!")
    print("The script will print the current ID being tested.")
    
    payloads = [
        ("1 Byte ON", [0x01]),
        ("1 Byte OFF", [0x00]),
        ("2 Bytes", [0x01, 0x00]),
        ("4 Bytes", [0x01, 0x00, 0x00, 0x00]), 
        ("Empty", [])
    ]
    
    for fid in range(256):
        if fid % 10 == 0:
            print(f"Testing ID 0x{fid:02X}...")
        
        for desc, pl in payloads:
            # Send burst
            for _ in range(3):
                run_test(ser, fid, pl)
            
            # If we just sent ON, send OFF immediately after to toggle
            if pl == [0x01]:
                run_test(ser, fid, [0x00])

    ser.close()
    print("\nScan complete.")

if __name__ == "__main__":
    main()
