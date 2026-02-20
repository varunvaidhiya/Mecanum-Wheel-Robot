#!/usr/bin/env python3
"""
Ultra-gentle ramp test.
Starts at 0.01 m/s (10mm/s) and ramps slowly up.
This avoids the brownout caused by all 4 motors starting at full power.
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

def send_motion(ser, vx, vy=0.0, w=0.0):
    """Send kinematics motion command."""
    CAR_TYPE = 1  # Mecanum X3
    payload = struct.pack('<bhhh', CAR_TYPE, int(vx*1000), int(vy*1000), int(w*1000))
    pkt = build_packet(0x12, payload)
    ser.write(bytearray(pkt))

def send_beep(ser, duration_ms=100):
    payload = struct.pack('<h', duration_ms)
    pkt = build_packet(0x02, payload)
    ser.write(bytearray(pkt))
    print(f"BEEP ({duration_ms}ms)")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error: {e}")
        return

    print("=== ULTRA-GENTLE RAMP TEST ===")
    print("All 4 motors should be connected.")
    print()
    print("Start Beep...")
    send_beep(ser, 100)
    time.sleep(1.0)

    # Set Car Type to Mecanum first
    payload = struct.pack('<b', 1)
    ser.write(bytearray(build_packet(0x15, payload)))
    time.sleep(0.2)

    print("Ramping from 0.01 to 0.15 m/s very slowly...")
    print("Watch for movement. Press Ctrl+C to stop early.")

    ramp_speeds = [0.01, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15]

    for target_vx in ramp_speeds:
        print(f"  Speed: {target_vx:.2f} m/s ...")
        # Hold each speed for 2 seconds (20 packets)
        for _ in range(20):
            send_motion(ser, vx=target_vx)
            time.sleep(0.1)

    print("Stopping...")
    for _ in range(5):
        send_motion(ser, 0.0, 0.0, 0.0)
        time.sleep(0.1)

    time.sleep(0.5)
    print("End Beep...")
    send_beep(ser, 200)

    ser.close()
    print("Done! Did it move?")

if __name__ == "__main__":
    main()
