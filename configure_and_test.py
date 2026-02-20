#!/usr/bin/env python3
"""
Board Configuration Fix:
1. Reads current board state (CAR_TYPE, Battery Voltage)
2. Sends SET_CAR_TYPE = 1 (Mecanum)
3. Reads again to verify it stuck
4. Then sends a SLOW forward motion to test

RUN THIS FIRST before any other test.
Requires: LiPo battery connected (USB power is NOT enough for motors!)
"""
import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def calculate_checksum(data):
    return (sum(data) + 5) & 0xFF

def build_packet(msg_type, payload):
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    packet = [HEAD, DEVICE_ID, 0, msg_type] + list(payload)
    packet[2] = len(packet) - 1
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    return packet

def read_state(ser, duration=1.0):
    """Read state packets from board for `duration` seconds."""
    all_data = bytearray()
    t_end = time.time() + duration
    while time.time() < t_end:
        if ser.in_waiting:
            all_data.extend(ser.read(ser.in_waiting))
        time.sleep(0.05)
    return all_data

def find_packet_type(data, target_type):
    """Find first packet of given type in raw bytes."""
    idx = 0
    while idx < len(data) - 2:
        if data[idx] == 0xFF and data[idx+1] == 0xFB:
            if idx + 3 >= len(data):
                break
            length = data[idx + 2]
            pkt_type = data[idx + 3]
            pkt_end = idx + 2 + length
            if pkt_end < len(data) and pkt_type == target_type:
                return bytes(data[idx:pkt_end+1])
            idx = pkt_end + 1
        else:
            idx += 1
    return None

def print_car_type(data):
    pkt = find_packet_type(data, 0x0D)
    if pkt and len(pkt) >= 5:
        car_type = pkt[4]
        types = {0: "NONE (Not configured!)", 1: "Mecanum X3 ✅", 2: "Tank/4WD",
                 3: "Ackermann", 4: "Omni-3wheel", 5: "Other"}
        print(f"  CAR_TYPE = {car_type} = {types.get(car_type, 'Unknown')}")
    else:
        print("  (No state packet received)")

def print_battery(data):
    pkt = find_packet_type(data, 0x0A)
    if pkt and len(pkt) >= 13:
        voltage_raw = struct.unpack_from('<H', pkt, 4)[0]
        # 0x7084 = 28804 appears in payload but that's not voltage...
        # Let's check offset 10 which is '0x70' '0x84' -> 0x8470 = 33904 /1000 = 33.9V? No...
        # Offset 4: 0x00 0x00 -> 0V
        # All zeros means no LiPo connected
        print(f"  Battery: raw={[hex(x) for x in pkt[4:-1]]}")
        if all(b == 0 for b in pkt[4:-1] if b != 0x70 and b != 0x84):
            print("  ⚠️  Battery = 0V --- NO LiPo connected! Motors need LiPo!")
    else:
        print("  (No battery packet received)")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error: {e}")
        return

    print("=== BOARD CONFIGURATION FIX ===")
    print()
    
    # Step 1: Read current state
    print("Step 1: Reading current board state...")
    data = read_state(ser, 1.5)
    print_car_type(data)
    print_battery(data)
    print()
    
    # Step 2: Send SET_CAR_TYPE = 1 (Mecanum)
    print("Step 2: Sending SET_CAR_TYPE = 1 (Mecanum)...")
    payload = struct.pack('<b', 1)
    pkt = build_packet(0x15, payload)
    print(f"  Packet: {[hex(x) for x in pkt]}")
    for _ in range(5):  # Send 5x to make sure it registers
        ser.write(bytearray(pkt))
        time.sleep(0.1)
    time.sleep(0.5)
    print()
    
    # Step 3: Read state again to verify
    print("Step 3: Verifying CAR_TYPE after SET command...")
    data2 = read_state(ser, 1.5)
    print_car_type(data2)
    print()
    
    # Step 4: Beep to confirm alive
    print("Step 4: Sending beep (you should hear it)...")
    payload = struct.pack('<h', 300)
    ser.write(bytearray(build_packet(0x02, payload)))
    time.sleep(0.5)
    
    # Step 5: Gentle motion test
    print("Step 5: Sending gentle forward motion (0.05 m/s) for 2 seconds...")
    print("   *** ENSURE LIIPO IS CONNECTED OR MOTORS WON'T MOVE ***")
    print()
    for _ in range(20):
        payload = struct.pack('<bhhh', 1, 50, 0, 0)  # 0.05 m/s
        pkt = build_packet(0x12, payload)
        ser.write(bytearray(pkt))
        time.sleep(0.1)

    # Stop
    payload = struct.pack('<bhhh', 1, 0, 0, 0)
    ser.write(bytearray(build_packet(0x12, payload)))
    time.sleep(0.3)
    
    # End beep
    payload = struct.pack('<h', 200)
    ser.write(bytearray(build_packet(0x02, payload)))
    print("End beep sent. Did you hear it?")
    
    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
