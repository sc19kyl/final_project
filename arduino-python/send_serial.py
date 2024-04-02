import serial
import time
import os
# Define the serial port and baud rate
ser = serial.Serial('COM5', 9600) # Change 'COM3' to match your Arduino's serial port

# Function to send PWM value to Arduino
def send_pwm(pwm_value):
    # Limit pwm_value between 0 and 255
    pwm_value = max(0, min(pwm_value, 255))
    # Send PWM value to Arduino
    ser.write(bytes([pwm_value]))
def clear_serial():
    ser.flushInput()  # Clear input buffer
    os.system('cls' if os.name == 'nt' else 'clear')  # Clear terminal screen
# Main loop
if __name__ == "__main__":
    try:
        while True:
            # Generate PWM signal from 0 to 255 with a step of 10
            for pwm in range(0, 256, 10):
                send_pwm(255)
                print("PWM:", pwm)
                 # Wait for a while
                # Read acknowledgment from Arduino
                print(ser.readline().strip().decode('latin1'))
                time.sleep(0.1)
            while ser.in_waiting:
                print(ser.readline().strip().decode('latin1'))
                
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print("\nExiting...")
        send_pwm(0)
        ser.close()  # Close serial connection