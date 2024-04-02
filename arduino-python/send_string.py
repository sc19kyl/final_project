import serial.tools.list_ports

# Get a list of available ports
ports = serial.tools.list_ports.comports()

# Print available ports
for idx, port in enumerate(ports, start=1):
    print(f"{idx}: {port}")

# Prompt user to select a port
selected_idx = input("Select Port (enter number): ")

try:
    # Convert input to integer
    selected_idx = int(selected_idx)
    
    # Check if the selected index is within the valid range
    if 1 <= selected_idx <= len(ports):
        selected_port = ports[selected_idx - 1].device
        selected_port2 = ports[selected_idx-2].device
        print("Selected port:", selected_port)
    else:
        print("Invalid port number. Please enter a valid port number.")
        exit()

except ValueError:
    print("Invalid input. Please enter a valid number.")
    exit()

# Create a serial instance
serialInst = serial.Serial(selected_port, 115200)
serialInst2 = serial.Serial(selected_port2, 115200)
# Main loop for sending and receiving data
while True:
    command = input("Enter two numbers separated by comma (e.g., 100,200): ")
    command += "\n"
    # Check if the command is 'exit' to break out of the loop
    if command.lower() == 'exit':
        break
    
    # Send the command over serial
    serialInst.write(command.encode('utf-8'))
    serialInst2.write(command.encode('utf-8'))

# Close the serial port
serialInst.close()
