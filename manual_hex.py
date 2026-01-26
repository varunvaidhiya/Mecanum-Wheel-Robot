import serial
import time
import binascii

PORT = '/dev/ttyUSB0'
BAUD = 115200

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(e)
        return
        
    print("=== MANUAL HEX SENDER ===")
    print("Enter hex string to send (e.g. 'FF FB 01 02').")
    print("Type 'exit' to quit.")
    
    while True:
        s = input("HEX> ")
        if s.lower() in ['exit', 'quit']:
            break
            
        try:
            data = bytes.fromhex(s)
            ser.write(data)
            print(f"Sent: {len(data)} bytes")
        except Exception as e:
            print(f"Error: {e}")
            
    ser.close()

if __name__ == "__main__":
    main()
