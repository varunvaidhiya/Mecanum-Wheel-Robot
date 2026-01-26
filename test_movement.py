
import serial
import time

def send_command(ser, cmd_str):
    print(f"Sending: {cmd_str.strip()}")
    ser.write(cmd_str.encode('utf-8'))
    time.sleep(2.0) # Wait to see movement

def main():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        print("Opened serial port.")
        
        # Format 1: Yahboom Python Driver default
        # MOTOR,FL,FR,RL,RR
        send_command(ser, "MOTOR,30,30,30,30\n")
        send_command(ser, "MOTOR,0,0,0,0\n")
        
        time.sleep(1)
        
        # Format 2: Omnibot C++ Driver
        # <FL,FR,RL,RR>
        send_command(ser, "<30,30,30,30>\n")
        send_command(ser, "<0,0,0,0>\n")
        
        time.sleep(1)
        
        # Format 3: With $ prefix
        send_command(ser, "$MOTOR,30,30,30,30\n")
        send_command(ser, "$MOTOR,0,0,0,0\n")

        ser.close()
        print("Done.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
