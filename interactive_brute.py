
import serial
import time
import struct

def chk(data):
    return sum(data) & 0xFF

def send_burst(ser, msg_type, payload_fmt, values, desc):
    try:
        payload = struct.pack(payload_fmt, *values)
    except:
        return

    # Length = Type(1) + Payload(N) + Check(1) + Len(1) = 3 + N (Standard)
    # Let's try the one that worked on simple_spin.py (Standard Yahboom)
    # Length = 2 + len(payload)
    
    length = 2 + len(payload)
    body = [length, msg_type] + list(payload)
    checksum = chk(body)
    packet = [0xFF, 0xFB] + body + [checksum]
    
    print(f"\n--> Sending: {desc}")
    print(f"    Packet: {[hex(b) for b in packet]}")
    
    # Send continuous burst for 2 seconds
    t_end = time.time() + 2.0
    while time.time() < t_end:
        ser.write(bytearray(packet))
        time.sleep(0.05)
    
    # Stop command (clean up)
    # Best effort stop
    ser.write(bytearray([0xFF, 0xFB, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x08]))

def main():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    except Exception as e:
        print(f"Error opening port: {e}")
        print("Stop ROS first.")
        return

    # Wakeup
    ser.write(b"INIT\n")
    time.sleep(0.5)

    tests = [
        (0x01, '<hhh', (300, 0, 0), "Type 0x01 (Car Run): 3 Shorts (Speed 300)"),
        (0x01, '<fff', (0.3, 0.0, 0.0), "Type 0x01 (Car Run): 3 Floats (Speed 0.3)"),
        (0x02, '<bbbb', (50, 50, 50, 50), "Type 0x02 (Motor Run): 4 Bytes (Speed 50)"),
        (0x02, '<hhhh', (300, 300, 300, 300), "Type 0x02 (Motor Run): 4 Shorts (Speed 300)"),
        (0x04, '<bbbb', (50, 50, 50, 50), "Type 0x04 (Control?): 4 Bytes (Speed 50)"),
    ]

    print("=== INTERACTIVE PROTOCOL TESTER ===")
    print("Press ENTER to run each test. Watch the robot!")
    print("If it moves, REMEMBER THE TEST NAME.\n")

    for t_type, t_fmt, t_vals, t_desc in tests:
        input(f"Ready to test: '{t_desc}'? (Press Enter)")
        send_burst(ser, t_type, t_fmt, t_vals, t_desc)
        print("Did it move? (Yes/No)") 
        # We don't capture input here to keep flow fast, user can just see output

    print("\n=== DONE ===")
    ser.close()

if __name__ == "__main__":
    main()
