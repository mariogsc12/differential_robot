import serial
import time

# Configure the serial port
ser = serial.Serial(
    port='COM3',       # Replace with your port name, e.g., '/dev/ttyUSB0' for Linux/Mac
    baudrate=115200,
    timeout=1          # Timeout for read operations
)

# Wait for the connection to initialize
time.sleep(2)

# Data to send
data = "lp1.00,rp1.00,"

# Write data to serial port
ser.write(data.encode('utf-8'))  # Convert string to bytes

# Optional: Close the serial port
ser.close()

