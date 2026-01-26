import serial
import time
import struct
import sys

# Configuration
PORT = '/dev/ttyUSB0'
BAUD = 115200

def chk(data):
    return sum(data) & 0xFF

def build_packet(func_id, payload):
    # Length = LenByte(1) + FuncID(1) + Payload(N) + Checksum(1)
    # Length Value = 1 + 1 + N + 1 = 3 + N
    # BUT, based on analysis:
    # 09 ... 0C ... 48
    # 09 (Len) = 1(Len) + 1(Func) + 6(Data) + 1(Check)
    # So Value = 2 + len(payload) + 1 = 3 + len(payload)?
    # Wait.
    # Pkt: FF FB 09 0C 2D 00 20 FF E8 FF 48
    # Len: 09.
    # Internal: 0C (1) + 2D..FF (6) + 48 (1). Total 8?
    # Length byte counts itself?
    # 09 (1) + 0C (1) + 6 (Data) + 1 (Check) = 9.
    # Yes. Length Value = 3 + len(payload).
    
    length_val = 3 + len(payload)
    body = [length_val, func_id] + list(payload)
    checksum = chk(body)
    return [0xFF, 0xFB] + body + [checksum]

def run_test(ser, func_id, payload_type, values, description):
    try:
        if payload_type == '3s':
            payload = struct.pack('<hhh', *values)
        elif payload_type == '4b':
            payload = struct.pack('<bbbb', *values)
        elif payload_type == '4s':
            payload = struct.pack('<hhhh', *values)
        else:
            return
    except:
        return

    pkt = build_packet(func_id, payload)
    
    print(f"\n[Test ID: 0x{func_id:02X} | {description}]")
    print(f"Sending: {[hex(x) for x in pkt]}")
    
    # Send for 1.5 seconds
    start = time.time()
    while time.time() - start < 1.5:
        ser.write(bytearray(pkt))
        time.sleep(0.05)
    
    # Stop command? (Best effort with same ID, zero payload)
    try:
        if payload_type == '3s':
            stop_pl = struct.pack('<hhh', 0, 0, 0)
        elif payload_type == '4b':
            stop_pl = struct.pack('<bbbb', 0, 0, 0, 0)
        elif payload_type == '4s':
            stop_pl = struct.pack('<hhhh', 0, 0, 0, 0)
        
        stop_pkt = build_packet(func_id, stop_pl)
        ser.write(bytearray(stop_pkt))
    except:
        pass

def main():
    print("Opening Serial Port...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except Exception as e:
        print(f"Error: {e}")
        return

    # Tests to run
    # 1. 3 Shorts (Standard Mecanum: Vx, Vy, Vz)
    # Scanning IDs 0x01 to 0x05
    # Speed: 150 (Reasonable for shorts - low but visible)
    
    ids_to_scan = [0x01, 0x02, 0x03, 0x04, 0x05]
    
    print("\n=== SCANNING 3 SHORTS (Vx, Vy, Vz) ===")
    for fid in ids_to_scan:
        input(f"Press ENTER to test ID 0x{fid:02X} (3 Shorts, Speed 150)...")
        run_test(ser, fid, '3s', (150, 0, 0), "Px: 150, 0, 0")
        
    print("\n=== SCANNING 4 BYTES (M1, M2, M3, M4) ===")
    for fid in ids_to_scan:
        input(f"Press ENTER to test ID 0x{fid:02X} (4 Bytes, Speed 50)...")
        run_test(ser, fid, '4b', (50, 50, 50, 50), "Px: 50, 50, 50, 50")

    print("\n=== SPECIAL: ID 0x01 with 3 Shorts OFFSET ===")
    # Maybe header is different? No, verified.
    # Maybe payload is Big Endian?
    input(f"Press ENTER to test ID 0x01 (3 Shorts, Big Endian)...")
    payload = struct.pack('>hhh', 150, 0, 0)
    pkt = build_packet(0x01, payload)
    print(f"Sending: {[hex(x) for x in pkt]}")
    for _ in range(20):
        ser.write(bytearray(pkt))
        time.sleep(0.05)
        
    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
