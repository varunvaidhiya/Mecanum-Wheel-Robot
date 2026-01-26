
import serial
import time
import binascii

def try_baud_rate(baud):
    print(f"Testing baud rate: {baud}")
    try:
        ser = serial.Serial('/dev/ttyUSB0', baud, timeout=1)
        start_time = time.time()
        while time.time() - start_time < 5:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                try:
                    text = data.decode('utf-8', errors='ignore')
                    print(f"Hex: {binascii.hexlify(data)}")
                    print(f"Text: {text}")
                except Exception:
                    print(f"Hex: {binascii.hexlify(data)}")
            time.sleep(0.1)
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Test sending INIT command
    try:
        print("Sending INIT command...")
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        ser.write(b"INIT\n")
        time.sleep(0.1)
        ser.write(b"MODE_MECANUM\n")
        time.sleep(0.1)
        ser.close()
    except Exception as e:
        print(f"Write error: {e}")
        
    try_baud_rate(115200)
    print("-" * 20)
    try_baud_rate(9600)
