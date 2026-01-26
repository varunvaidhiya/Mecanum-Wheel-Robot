import serial
import time
import binascii

PORT = '/dev/ttyUSB0'
BAUD_RATES = [115200, 9600, 57600, 38400]

def sniff(baud):
    print(f"\n--- Sniffing at {baud} baud for 5 seconds ---")
    try:
        ser = serial.Serial(PORT, baud, timeout=0.1)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    start_time = time.time()
    buffer = b""
    
    while time.time() - start_time < 5:
        data = ser.read(100) # Read up to 100 bytes
        if data:
            buffer += data
            # Print chunks as they come in
            hex_data = binascii.hexlify(data, ' ').decode('utf-8')
            print(f"Rx: {hex_data}")
        time.sleep(0.01)
    
    print(f"Total bytes received: {len(buffer)}")
    if len(buffer) > 0:
        print("Sample raw: " + str(buffer[:50]))
    ser.close()

if __name__ == "__main__":
    print(f"Starting Sniffer on {PORT}...")
    for b in BAUD_RATES:
        sniff(b)
    print("\nSniffing Complete.")
