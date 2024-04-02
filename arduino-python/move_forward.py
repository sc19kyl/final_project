import serial.tools.list_ports
import time

try:
    # Get a list of available ports
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("No serial ports found.")
        exit()
    
    # Print available ports
    print("Available ports:")
    for idx, port in enumerate(ports, start=1):
        print(f"{idx}: {port}")
    
    # Create serial instances for all available ports and send the command
    command = "50,50"
    command += "\n"
    for port in ports:
        serialInst = serial.Serial(port.device, 115200)
        serialInst.write(command.encode('utf-8'))
        serialInst.close()  # Close the port after sending the command
        
    print("Command sent to all devices.")

except Exception as e:
    print("An error occurred:", e)
