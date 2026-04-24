import serial
import sys

# Configuration
BLUETOOTH_PORT = "/dev/tty.MINUNHC05"  # Change this based on what device you're using. this if configured to run on MacOS
BAUD_RATE = 9600
TIMEOUT = 1

def main():
    try:
        # Open the Bluetooth serial port
        ser = serial.Serial(BLUETOOTH_PORT, BAUD_RATE, timeout=TIMEOUT)
        ser.write("AT\r\n".encode())  # Test command to check connection
        print(f"Connected to {BLUETOOTH_PORT} at {BAUD_RATE} baud")
        print("Reading telemetry data... (Press Ctrl+C to stop)\n")
        
        while True:
            # Read a line from the Bluetooth module
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[TELEMETRY] {line}")
    
    except serial.SerialException as e:
        print(f"Error: Could not open {BLUETOOTH_PORT}")
        print(f"Details: {e}")
        print("\nMake sure:")
        print("1. HC-05 is paired to your computer")
        print("2. You've identified the correct COM port (check Device Manager on Windows)")
        sys.exit(1)
    
    except KeyboardInterrupt:
        print("\n\nClosing connection...")
        ser.close()
        print("Done!")

if __name__ == "__main__":
    main()