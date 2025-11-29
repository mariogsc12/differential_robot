import serial
import time
import argparse
import sys

# Import serial port from terminal
parser = argparse.ArgumentParser()
parser.add_argument(
    "--port", 
    type=str,
    required=False,
    default="COM3", 
    help="Serial port name"
)
parser.add_argument(
    "--msg", 
    type=str,
    required=False,
    default="lp0.00,rp0.00,", 
    help="Serial port message"
)
args = parser.parse_args()

if not args.port or not args.msg:
    print("ERROR - Serial port or message undefined")
    sys.exit(1)
else:
    serial_port = str(args.port)
    serial_data = str(args.msg)

# Configure the serial port
ser = serial.Serial(
    port=serial_port,       
    baudrate=115200,
    timeout=1          # Timeout for read operations
)

# Wait for the connection to initialize
time.sleep(2)

# Write data to serial port
ser.write(serial_data.encode('utf-8'))  # Convert string to bytes

# Optional: Close the serial port
ser.close()

