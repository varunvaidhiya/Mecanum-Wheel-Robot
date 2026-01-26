
import serial
import time
import struct

class Rosmaster:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.header = [0xFF, 0xFB]

    def _calculate_checksum(self, data):
        return sum(data) & 0xFF

    def send_packet(self, msg_type, payload):
        # Length = 1 (Type) + Len(Payload) + 1 (Checksum) ? 
        # Or Length = 1 (Len) + 1 (Type) + Len(Payload) + 1 (Checksum) ?
        # Based on typical Yahboom standard:
        # Len = Type + PayloadBytes + Checksum (1 byte)
        
        length = 1 + len(payload) + 1
        body = [length, msg_type] + list(payload)
        checksum = self._calculate_checksum(body)
        packet = self.header + body + [checksum]
        
        print(f"Sending: {[hex(b) for b in packet]}")
        self.ser.write(bytearray(packet))

    def set_car_motion(self, vx, vy, vz):
        # Type 0x01
        # Payload: 3 shorts (vx, vy, vz)
        # Scale: Usually multiplied by 1000 or similar, but let's try raw
        # Assuming int16 (2 bytes)
        
        # Yahboom often uses 0x01 for mecnaum motion
        payload = struct.pack('<hhh', int(vx), int(vy), int(vz))
        self.send_packet(0x01, payload)

    def set_motor(self, m1, m2, m3, m4):
        # Type 0x02 or 0x01
        # Payload: 4 signed bytes
        payload = struct.pack('<bbbb', int(m1), int(m2), int(m3), int(m4))
        self.send_packet(0x02, payload) # Trying 0x02 for direct motor

    def close(self):
        self.ser.close()

if __name__ == '__main__':
    bot = Rosmaster()
    print("Testing set_car_motion (Type 0x01)...")
    bot.set_car_motion(200, 0, 0)
    time.sleep(1)
    bot.set_car_motion(0, 0, 0)
    time.sleep(1)
    
    print("Testing set_motor (Type 0x02)...")
    bot.set_motor(30, 30, 30, 30)
    time.sleep(1)
    bot.set_motor(0, 0, 0, 0)
    
    bot.close()
