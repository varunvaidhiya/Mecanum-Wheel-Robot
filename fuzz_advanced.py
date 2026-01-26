
import serial
import time
import struct

def checksum_sum(data):
    return sum(data) & 0xFF

def checksum_xor(data):
    chk = 0
    for b in data:
        chk ^= b
    return chk

def create_packet(msg_type, payload, algo='sum'):
    # Header: FF FB
    # Length: 1 (Len) + 1 (Type) + Len(Payload) + 1 (Checksum) ?
    # Let's try standard: Length = Type + Payload + Checksum (so len(payload)+2)
    # OR Length = Total bytes including Header?
    
    # Based on the read packet: FF FB 0A 0A ... 77 8B
    # Len 0A (10). Type 0A (10). Payload 6 zeros.
    # 1 (Len) + 1 (Type) + 6 (Payload) + 2 (Check+Tail?) = 10? No.
    # 1 (Type) + 6 (Payload) + 1 (Check) = 8.
    
    # Let's guess: Length byte = (1 Type + N Payload)
    
    body = [msg_type] + list(payload)
    length = len(body) # Length of body only?
    
    # Construct various packet formats to test
    
    packets = []
    
    # Format A: FF FB Len Type Payload Check (Sum)
    # Len = 1(Type) + N(Payload)
    pkt_a = [0xFF, 0xFB, len(body), msg_type] + list(payload)
    chk_a = checksum_sum(pkt_a[2:]) # Checksum of Len+Type+Payload
    packets.append(bytearray(pkt_a + [chk_a]))

    # Format B: FF FB Len Type Payload Check (Sum of BODY)
    chk_b = checksum_sum(body)
    packets.append(bytearray([0xFF, 0xFB, len(body), msg_type] + list(payload) + [chk_b]))

    # Format C: FF FB Len Type Payload Check (Sum of ALL)
    # Len = Total length
    total_len = 2 + 1 + 1 + len(payload) + 1
    pkt_c = [0xFF, 0xFB, total_len, msg_type] + list(payload)
    chk_c = checksum_sum(pkt_c)
    packets.append(bytearray(pkt_c + [chk_c]))
    
    return packets

def main():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        print("Opened serial port.")
        
        # Payload for "Move Forward"
        # 3 shorts (Vx=200, Vy=0, Omega=0)
        payload = struct.pack('<hhh', 200, 0, 0)
        
        while True:
            print("Sending batches...")
            packets = create_packet(0x01, payload)
            for i, pkt in enumerate(packets):
                print(f"Format {i}: {[hex(b) for b in pkt]}")
                for _ in range(5): # Send burst
                    ser.write(pkt)
                    time.sleep(0.05)
                time.sleep(0.5)
            
            # Also try 0x02 (Motor control) with 4 chars
            payload_motor = struct.pack('<bbbb', 30,30,30,30)
            packets_motor = create_packet(0x02, payload_motor)
            for i, pkt in enumerate(packets_motor):
                print(f"Motor Format {i}: {[hex(b) for b in pkt]}")
                for _ in range(5):
                    ser.write(pkt)
                    time.sleep(0.05)
                time.sleep(0.5)

            time.sleep(2)
            
    except KeyboardInterrupt:
        ser.close()
        print("Done.")

if __name__ == "__main__":
    main()
