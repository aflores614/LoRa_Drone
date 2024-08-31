from pymavlink import mavutil
import serial
import time
import sys
from connect_to_vehicle import connect_to_vehicle
from arm_drone import arm_drone, is_armed
from disarm_drone import disarm_drone
from lora import send_command, get_address, get_network, read_command

serial_port = '/dev/ttyUSB1'
baud_rate = 115200  # Default baud rate for RYLR998
GC_Address = 1

ser = serial.Serial(serial_port, baud_rate, timeout=1)

try:
    ADDRESS = get_address(ser)
    print("The ADDRESS is:", ADDRESS)
    
    network = get_network(ser)
    print("The network is:", network)
    
    while True:  # Establish communication between Drone and Ground Control
        send_command(ser, GC_Address, "INPUT.Connection y/n")
        response = read_command(ser)
        print(response)
        
        if response == 'y':
            break
        elif response == 'n':
            sys.exit()
        else:
            send_command(ser, GC_Address, "ERROR.Invalid input")
    
    master = connect_to_vehicle()
    
    if master:
        send_command(ser, GC_Address, "INFO.Vehicle Connected")

        while True:
            send_command(ser, GC_Address, "INPUT.Arm Drone y/n: ")
            arm_response = read_command(ser)
            
            if arm_response == 'y':
                break
            elif arm_response == 'n':
                sys.exit()
            else:
                send_command(ser, GC_Address, "ERROR.Invalid input")      
        
        while True:
            arm_drone(master)
            if is_armed(master):
		
                send_command(ser, ADDRESS, "INFO.Drone is armed!")
                break            
            else:
                send_command(ser, ADDRESS, "INFO.Drone is not armed")      
            time.sleep(3)
        
        time.sleep(5)
        disarm_drone(master)

except KeyboardInterrupt:
    print("Can't connect")

ser.close()
