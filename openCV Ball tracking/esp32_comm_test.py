import serial
import time

# Configure serial port
ser = serial.Serial('COM6', 460800, timeout=1)  # Adjust port and baudrate as needed

def send_data(data):
    # Send data over serial
    ser.write(data.encode())
    print("Data sent:", data)

# Example usage
while True:
    user_input = input("Enter data to send to ESP32: ")
    send_data(user_input)
    time.sleep(1)  # Delay between sending data
