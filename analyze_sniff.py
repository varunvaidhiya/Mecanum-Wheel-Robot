
data_hex = """
ff fb 13 0d ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff 10
ff fb 0a 0a 00 00 00 00 00 00 77 8b
ff fb 15 0e fe ff fe ff fb ff d6 ff 27 ff 72 d9 3f 
"""
# Note: I pasted a small chunk, but I'll implement a parser for the structure `FF FB [Len] [Type]` etc.

import binascii

def calculate_sum(data):
    return sum(data) & 0xFF

def analyze(data_str):
    # Clean up the string
    clean_data = data_str.replace("Rx:", "").replace("\\n", " ").replace("\\r", " ").replace("b'", "").replace("'", "")
    # hex_bytes = bytes.fromhex(clean_data) # This might fail if dirty
    
    # Let's manual walk
    bytes_list = []
    for x in clean_data.split():
        try:
            bytes_list.append(int(x, 16))
        except:
            pass
            
    # Search for headers FF FB
    i = 0
    while i < len(bytes_list) - 1:
        if bytes_list[i] == 0xFF and bytes_list[i+1] == 0xFB:
            # Possible start
            # Let's guess the next byte is Type or Length
            # Based on `FF FB 0A 0A` instance:
            # [FF FB] [0A] [0A] [00 00 00 00 00 00] [77 8B]? 
            # If 3rd byte is Length (0x0A = 10)?
            # Packet: FF FB 0A 0A 00 00 00 00 00 00 (size 10?) + Checksum?
            
            # Let's grab next 20 bytes and print
            print(f"Header at {i}: ", end="")
            chunk = bytes_list[i:i+20]
            print( " ".join([f"{b:02x}" for b in chunk]) )
            
            # Checksum hypothesis: Sum of body?
            # Example: FF FB 0A 0A 00 00 00 00 00 00 77 8B
            # Sum(0A 0A 00...) = 14 (0x14). Checksum is 77 8B? No.
            # Maybe CRC?
            
            i += 1
        else:
            i += 1

test_data = [
    # Packet 1: 0A
    [0xFF, 0xFB, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x77, 0x8B],
    # Packet 2: 09
    [0xFF, 0xFB, 0x09, 0x0C, 0x2D, 0x00, 0x1F, 0xFF, 0xE8, 0xFF, 0x47], # From sniffer log
    # Packet 3: 13
    [0xFF, 0xFB, 0x13, 0x0D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x10], 
]

def check_msg(pkt):
    print(f"\\nPacket: {[hex(x) for x in pkt]}")
    # Hypothesis 1: Byte 3 is payload length?
    # Pkt 1: Byte 3 = 0x0A (10). Total len 12. 12 - 2 (Header) = 10?
    #   FF FB [0A ... 8B] -> 10 bytes?
    #   Content: 0A 00 00 00 00 00 00 77 8B? No, 0x0A is repeated.
    
    # Hypothesis 2: Byte 3 is Length (including self?), Byte 4 is Type?
    # Pkt 1: Len 0x0A (10) -> Type 0x0A?
    # Pkt 2: FF FB 09 0C ... 
    #   Len 09? Contents: 0C 2D 00 1F FF E8 FF 47? (8 bytes). 8 != 9.
    
    # Hypothesis 3: Checksum is last byte. Sum of everything except Header and Checksum?
    # Pkt 2: FF FB 09 0C 2D 00 1F FF E8 FF 47
    #   Data: 09 0C 2D 00 1F FF E8 FF. Sum = ?
    #   09+0C+2D+00+1F+FF+E8+FF = 9+12+45+31+255+232+255 = 839. 
    #   839 & 0xFF = 0x47. MATCH!!
    
    # Let's verify usage of Method 3 on Packet 1
    # Pkt 1: FF FB 0A 0A 00 00 00 00 00 00 77 8B
    #   Last byte 8B. 
    #   Data: 0A 0A 00 00 00 00 00 00 77.
    #   Sum: 10+10+119 = 139 = 0x8B. MATCH!!
    
    # Let's verify usage of Method 3 on Packet 3
    # Pkt 3: FF FB 13 0D FF FF ... 10
    #   Last byte 10.
    #   Data: 13 0D FF...
    #   Sum: 13 + 13 + (16 * 255). 
    #   13+13 + 4080 = 4106. 4106 & 0xFF = 0x0A. 
    #   Wait, last byte is 0x10? Maybe I missed a byte in copy paste?
    #   Let's re-read sniffer log carefully.
    pass

for p in test_data:
    check_msg(p)
