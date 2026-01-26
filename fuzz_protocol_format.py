import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

def list_checksums(data):
    s = sum(data) & 0xFF
    return [
        s,                  # Standard Sum
        (s + 1) & 0xFF,     # Sum + 1?
        (~s) & 0xFF,        # Inverted
        (s ^ 0xFF) & 0xFF,  # XOR FF
        0x00,               # Zero
    ]

def send_packet(ser, length_byte, func_id, payload, cs_algo_idx):
    # Construct body
    # Header: FF FB
    # Body: [Length, Type, Payload...] ? 
    # Or [Type, Payload...] and Length is just a byte?
    
    # Let's assume structure: FF FB [L] [T] [P...] [CS]
    # We iterate L (Length Byte)
    
    body = [length_byte, func_id] + list(payload)
    
    # Calculate checksum based on body
    checksums = list_checksums(body)
    cs = checksums[cs_algo_idx % len(checksums)]
    
    packet = [0xFF, 0xFB] + body + [cs]
    
    ser.write(bytearray(packet))
    # time.sleep(0.01) # Fast fuzz

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(e)
        return

    print("=== PROTOCOL FORMAT FUZZER (BUZZER) ===")
    print("Trying to make the robot BEEP.")
    print("If you hear a beep, NOTE THE 'L=' value output.")
    
    # Target: Buzzer ON (ID=0x03 or 0x02? Let's try 0x03 first based on success of other yahboom bots)
    # Payload: 0x01 (ON)
    
    func_ids = [0x03] # Buzzer usually
    payload = [0x01]  # ON
    
    for fid in func_ids:
        print(f"\nFuzzing Target ID: 0x{fid:02X} (Buzzer ON)")
        
        # Iterate Length Values 0 to 32
        for l_val in range(32):
            print(f"Testing L={l_val}...", end='\r')
            
            # Send with various checksums
            for cs_idx in range(5):
                # Send burst
                for _ in range(5): 
                    send_packet(ser, l_val, fid, payload, cs_idx)
                time.sleep(0.02)
                
            # Also try alternate structure: FF FB [L] [T] [P...] [CS]
            # Where L = Length of (Type+Payload)?
            
    print("\n\nDid you hear a beep? (If not, we might try ID 0x06 or others)")
    
    # Try ID 0x06 (Some bots use 0x06 for buzzer)
    print("\nTrying ID 0x06 (Alternate Buzzer)...")
    for l_val in range(32):
         print(f"Testing L={l_val}...", end='\r')
         for cs_idx in range(5):
            for _ in range(5):
                send_packet(ser, l_val, 0x06, payload, cs_idx)
            time.sleep(0.02)

    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
