
def brute_force_checksum(packet_hex_str):
    # Convert hex string to bytes
    data = bytes.fromhex(packet_hex_str)
    
    # Assume packet is FF FB Len Type ... Check
    # Or FF FB ... Check
    
    # Captured: fffb150efaff01000400220044004bd9950092b80ab84c
    # Header: FF FB
    # Valid Data: 15 0E FA FF 01 00 04 00 22 00 44 00 4B D9 95 00 92 B8 0A B8
    # Checksum: 4C
    
    # Extract candidate ranges
    # Full packet without header and checksum
    full_body = data[2:-1] 
    
    print(f"Analyzing Packet: {packet_hex_str}")
    print(f"Body: {[hex(b) for b in full_body]}")
    target_checksum = data[-1]
    print(f"Target Checksum: {hex(target_checksum)}")
    
    # Algo 1: Sum
    s = sum(full_body) & 0xFF
    print(f"Algo Sum: {hex(s)} {'MATCH' if s == target_checksum else ''}")
    
    # Algo 2: Sum + Length
    # If Length is not included in body?
    # Length byte is data[2].
    # Try sum of data[2:-1] (includes length)
    s_inc_len = sum(data[2:-1]) & 0xFF
    print(f"Algo Sum(Len+Body): {hex(s_inc_len)} {'MATCH' if s_inc_len == target_checksum else ''}")

    # Algo 3: XOR
    x = 0
    for b in data[2:-1]:
        x ^= b
    print(f"Algo XOR: {hex(x)} {'MATCH' if x == target_checksum else ''}")

    # Algo 4: Sum of payload only (excluding Len and Type?)
    # Type is data[3]
    if len(data) > 4:
        s_payload = sum(data[4:-1]) & 0xFF
        print(f"Algo Sum(Payload): {hex(s_payload)} {'MATCH' if s_payload == target_checksum else ''}")

if __name__ == "__main__":
    # Packets captured from debug_serial.py
    packets = [
        "fffb150efaff01000400220044004bd9950092b80ab84c",
        "fffb130dffffffffffffffffffffffffffffffff10",
        "fffb090cf1ff1b00dff9f8",
        "fffb150e0500060000000500140085d950fb92b8c4b2b0"
    ]
    
    for p in packets:
        brute_force_checksum(p)
        print("-" * 20)
