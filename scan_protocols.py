
import serial
import time
import struct

def chk_sum(data):
    return sum(data) & 0xFF

def chk_xor(data):
    x = 0
    for b in data:
        x ^= b
    return x

def send_packet(ser, header, msg_type, payload, chk_algo, desc):
    # Construct packet
    # Length strategy: 
    # Yahboom typically (Len, Type, Payload, Check)
    # Len = Type + PayloadBytes + Check ?? Or just Type + Payload?
    # Let's assume standard: Len = Type(1) + Payload(N) + Check(1)
    
    # Body [Len, Type, Payload]
    # Checksum on Body? Or Header+Body?
    
    # Common Yahboom:
    # HeaderHeader Len Type Payload Check
    # Check = Sum(Len+Type+Payload)
    
    body = [0, msg_type] + list(payload)
    #body[0] = 1 + len(payload) + 1 # With checksum
    body[0] = 1 + len(payload) # Without checksum in length count?
    # Let's try "Len includes Checksum" first which is common.
    
    body_len_inc = [1 + len(payload) + 1, msg_type] + list(payload)
    chk_inc = chk_sum(body_len_inc) if chk_algo == 'sum' else chk_xor(body_len_inc)
    pkt_inc = header + body_len_inc + [chk_inc]
    
    # Try "Len excludes Checksum"
    body_len_exc = [1 + len(payload), msg_type] + list(payload)
    chk_exc = chk_sum(body_len_exc) if chk_algo == 'sum' else chk_xor(body_len_exc)
    pkt_exc = header + body_len_exc + [chk_exc]

    print(f"  > [{desc}] Sending burst...")
    # Send both variants to be sure
    for _ in range(10):
        ser.write(bytearray(pkt_inc))
        time.sleep(0.02)
    time.sleep(0.5)

def scan():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        print("Opened serial port.")
        
        # Payloads
        # 4 motors forward (Type 0x01 or 0x02)
        # 30 speed
        p_motor = struct.pack('<bbbb', 40, 40, 40, 40)
        
        # Car Run (Vx, Vy, Vz)
        p_car = struct.pack('<hhh', 300, 0, 0)
        
        protocols = [
            ([0xFF, 0xFB], 'sum', "FF FB (Sum)"),
            ([0xFF, 0xFB], 'xor', "FF FB (Xor)"),
            ([0xFF, 0xFF], 'sum', "FF FF (Sum)"),
            ([0xFF, 0x00], 'sum', "FF 00 (Sum)"), # Rare
            ([0xAA, 0x55], 'sum', "AA 55 (Sum)"),
        ]
        
        for head, algo, name in protocols:
            print(f"\n=== TESTING PROTOCOL: {name} ===")
            
            # Test Type 0x01 Motor (4 bytes)
            send_packet(ser, head, 0x01, list(p_motor), algo, "Type 0x01 (4 Motors)")
            
            # Test Type 0x02 Motor (4 bytes)
            send_packet(ser, head, 0x02, list(p_motor), algo, "Type 0x02 (4 Motors)")

            # Test Type 0x01 Car (3 Shorts)
            send_packet(ser, head, 0x01, list(p_car), algo, "Type 0x01 (Car VxVyVz)")
            
            time.sleep(2) # Paus between protocols

        print("\nScan Complete.")
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    scan()
