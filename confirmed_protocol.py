import serial
import time
import struct

PORT = '/dev/ttyUSB0'
BAUD = 115200

class RosmasterProtocol:
    def __init__(self):
        self.HEAD = 0xFF
        self.DEVICE_ID = 0xFC
        self.COMPLEMENT = 257 - self.DEVICE_ID # 5
        self.CAR_TYPE = 1 # X3
        
        self.FUNC_BEEP = 0x02
        self.FUNC_MOTION = 0x12
        self.FUNC_MOTOR = 0x10
        self.FUNC_SET_CAR_TYPE = 0x15

    def calculate_checksum(self, packet):
        # packet includes HEAD, ID, LEN, TYPE, DATA...
        # checksum = sum(cmd, self.__COMPLEMENT) & 0xff
        total = sum(packet) + self.COMPLEMENT
        return total & 0xFF

    def build_packet(self, func_id, payload):
        # Structure: HEAD, ID, LEN, FUNC, PAYLOAD..., CS
        # Len = Count from ID to end of Payload
        # Dummy len first
        
        # ID(1) + Len(1) + Func(1) + Payload(N)
        length_val = 1 + 1 + 1 + len(payload)
        
        packet = [self.HEAD, self.DEVICE_ID, length_val, func_id] + list(payload)
        
        cs = self.calculate_checksum(packet)
        packet.append(cs)
        return packet

    def set_beep(self, on_time_ms):
        # Value is short (2 bytes)
        # on_time_ms = 1 for continuous, 0 for off.
        payload = struct.pack('<h', int(on_time_ms))
        payload_list = list(payload)
        
        # Line 396: cmd = [HEAD, ID, 0x05, FUNC, val0, val1]
        # My build_packet calculates len automatically.
        # ID(1)+Len(1)+Func(1)+Data(2) = 5. Matches 0x05.
        
        return self.build_packet(self.FUNC_BEEP, payload_list)

    def set_motion(self, vx, vy, vz):
        # set_car_motion: HEAD, ID, LEN, FUNC, TYPE, VX(2), VY(2), VZ(2)
        # Payload: TYPE + VX + VY + VZ
        # Type is self.CAR_TYPE (1)
        
        # Scale: * 1000
        vx_int = int(vx * 1000)
        vy_int = int(vy * 1000)
        vz_int = int(vz * 1000)
        
        payload = struct.pack('<bhhh', self.CAR_TYPE, vx_int, vy_int, vz_int)
        return self.build_packet(self.FUNC_MOTION, list(payload))

def main():
    print("Opening Serial...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error: {e}")
        return

    proto = RosmasterProtocol()

    print("--- TEST 1: BEEPER ---")
    print("Sending Beep ON (100ms)...")
    pkt = proto.set_beep(100) # 100ms
    print(f"Sending: {[hex(x) for x in pkt]}")
    ser.write(bytearray(pkt))
    time.sleep(1)

    print("--- TEST 2: MOTION ---")
    print("Moving FORWARD 0.3 m/s...")
    pkt_move = proto.set_motion(0.3, 0.0, 0.0)
    print(f"Sending: {[hex(x) for x in pkt_move]}")
    
    # Send loop for 2 seconds
    t_end = time.time() + 2
    while time.time() < t_end:
        ser.write(bytearray(pkt_move))
        time.sleep(0.05)
        
    print("Stopping...")
    pkt_stop = proto.set_motion(0.0, 0.0, 0.0)
    ser.write(bytearray(pkt_stop))
    
    print("--- TEST 3: READ HEARTBEAT ---")
    print("Listening for data (Voltage/State)...")
    start = time.time()
    while time.time() - start < 5.0:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(f"RX: {[hex(x) for x in data]}")
        time.sleep(0.1)

    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
