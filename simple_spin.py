
import serial
import time
import struct

def calculate_checksum(data):
    return sum(data) & 0xFF

def send_packet(ser, msg_type, payload):
    # Construct packet: FF FB [Len] [Type] [Payload...] [Checksum]
    # Length = 2 + len(payload) (Type + Payload + Checksum count?)
    # Try Length = Type(1) + Payload(N) + Checksum(1) => 2 + len(payload)
    body = [2 + len(payload), msg_type] + list(payload)
    checksum = calculate_checksum(body)
    packet = [0xFF, 0xFB] + body + [checksum]
    
    # print(f"Sending: {[hex(b) for b in packet]}")
    ser.write(bytearray(packet))

def main():
    port = '/dev/ttyUSB0'
    baud = 115200
    
    print(f"Opening {port} at {baud}...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print(f"Error opening port: {e}")
        print("Make sure to STOP all ROS nodes first! (Ctrl+C in other terminals)")
        return

    print("Initialization sent (if needed)...")
    # Some boards need initialization
    ser.write(b"INIT\n") 
    time.sleep(0.5)

    print("--- TESTING MOTORS ---")
    print("Sending Forward Command for 2 seconds...")
    
    # Send continuous stream to prevent timeout safety measures
    start_time = time.time()
    while time.time() - start_time < 2.0:
        # Type 0x02 Motor Control
        # 4 bytes: M1, M2, M3, M4 speed (-100 to 100)
        # Move forward: All positive? Or Mecanum mix?
        # Let's try 30 speed on all.
        payload = struct.pack('<bbbb', 30, 30, 30, 30)
        send_packet(ser, 0x02, payload)
        time.sleep(0.05) # 20Hz
        
    print("Stopping...")
    stop_payload = struct.pack('<bbbb', 0, 0, 0, 0)
    for _ in range(5):
        send_packet(ser, 0x02, stop_payload)
        time.sleep(0.05)
        
    ser.close()
    print("Done. Did the wheels spin?")

if __name__ == "__main__":
    main()
