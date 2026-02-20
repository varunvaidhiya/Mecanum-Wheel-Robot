#!/usr/bin/env python3
"""
Board State Decoder.
Reads and decodes the board's heartbeat packets to determine:
  - Current CAR_TYPE configuration
  - Battery voltage
  - Motor encoder feedback
  - IMU data
This reveals whether the board 'knows' it's a Mecanum robot.
"""
import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def parse_heartbeat(data):
    """Parse all FF FB packets in the received data."""
    idx = 0
    packets = []
    while idx < len(data) - 2:
        if data[idx] == 0xFF and data[idx+1] == 0xFB:
            if idx + 2 >= len(data):
                break
            length = data[idx + 2]
            pkt_end = idx + 2 + length
            if pkt_end >= len(data):
                break
            pkt = data[idx:pkt_end+1]
            packets.append(pkt)
            idx = pkt_end + 1
        else:
            idx += 1
    return packets

def decode_packet(pkt):
    """Decode a single board heartbeat packet."""
    if len(pkt) < 4:
        return
    
    # Structure: FF FB LEN TYPE [PAYLOAD...] CS
    pkt_type = pkt[3]
    payload = pkt[4:-1]
    
    if pkt_type == 0x0A:
        # Battery Voltage packet
        if len(payload) >= 8:
            try:
                voltage_raw = struct.unpack_from('<H', payload, 0)[0]
                print(f"  TYPE 0x0A (Battery):")
                print(f"    Raw voltage bytes: {[hex(x) for x in payload[:2]]}")
                print(f"    Voltage raw: {voltage_raw} -> {voltage_raw/1000:.2f}V (if in mV)")
                print(f"    Voltage raw: {voltage_raw} -> {voltage_raw/100:.2f}V (if in 10mV)")
                print(f"    Full payload: {[hex(x) for x in payload]}")
            except:
                print(f"  TYPE 0x0A (Battery): {[hex(x) for x in payload]}")

    elif pkt_type == 0x0C:
        # Motor speed feedback
        print(f"  TYPE 0x0C (Motor Speeds):")
        if len(payload) >= 8:
            try:
                m1, m2, m3, m4 = struct.unpack_from('<hhhh', payload, 0)
                print(f"    M1={m1}, M2={m2}, M3={m3}, M4={m4}")
            except:
                print(f"    payload: {[hex(x) for x in payload]}")
        
    elif pkt_type == 0x0D:
        # State/Config packet - MOST IMPORTANT
        print(f"  TYPE 0x0D (Board State - IMPORTANT):")
        print(f"    Full payload ({len(payload)} bytes): {[hex(x) for x in payload]}")
        if len(payload) >= 1:
            car_type = payload[0]
            print(f"    CAR_TYPE = {car_type} ", end="")
            types = {0: "(NONE/Default)", 1: "Mecanum/X3", 2: "Tank/4WD", 
                     3: "Ackermann", 4: "Omni-3wheel", 5: "Other"}
            print(types.get(car_type, "(Unknown)"))

    elif pkt_type == 0x0E:
        # Encoder/Odometry
        print(f"  TYPE 0x0E (Encoder Odom):")
        if len(payload) >= 12:
            try:
                m1, m2, m3, m4 = struct.unpack_from('<hhhh', payload, 0)
                print(f"    Encoder: M1={m1}, M2={m2}, M3={m3}, M4={m4}")
                print(f"    Full payload: {[hex(x) for x in payload]}")
            except:
                print(f"    payload: {[hex(x) for x in payload]}")
    else:
        print(f"  TYPE {hex(pkt_type)}: {[hex(x) for x in payload]}")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error: {e}")
        return

    print("=== BOARD STATE READER ===")
    print("Listening for 5 seconds to read board configuration...")
    print()
    
    all_data = bytearray()
    start = time.time()
    while time.time() - start < 5.0:
        if ser.in_waiting:
            d = ser.read(ser.in_waiting)
            all_data.extend(d)
        time.sleep(0.05)
    
    print(f"Total bytes received: {len(all_data)}")
    print()
    
    # Parse all packets
    packets = parse_heartbeat(all_data)
    print(f"Parsed {len(packets)} FF FB packets.\n")
    
    # Show unique packet types
    seen_types = {}
    for pkt in packets:
        if len(pkt) >= 4:
            pkt_type = pkt[3]
            if pkt_type not in seen_types:
                seen_types[pkt_type] = pkt  # First occurrence
    
    print("=== UNIQUE PACKET TYPES (one example each) ===")
    for pkt_type, pkt in sorted(seen_types.items()):
        print(f"\nPacket type {hex(pkt_type)} ({len(pkt)} bytes):")
        decode_packet(pkt)
    
    ser.close()

if __name__ == "__main__":
    main()
