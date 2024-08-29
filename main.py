import serial
import time
from connect_to_vehicle import connect_to_vehicle
from arm_drone import arm_drone
from arm_drone import is_armed
from disarm_drone import disarm_drone
from lora import send_command,get_address,get_network,read_command

serial_port = '/dev/ttyUSB1'
baud_rate = 115200  # Default baud rate for RYLR998

ser = serial.Serial(serial_port, baud_rate, timeout=1)

try:
    ADDRESS = get_address(ser)
    print("The ADDRESS is: ", ADDRESS)
    network = get_network(ser)
    print("The network is: ", network)
    send_command(ser, ADDRESS, "Connection Test y/n")

    while True:
        reponse = read_command(ser)
        if(reponse == 'y'):
            break
        else:
            time.sleep(0.5)
    master = connect_to_vehicle()
    if master:
        send_command(ser, ADDRESS, "Vehicle Connect")
        send_command(ser, ADDRESS,"Arm the Drone y/n")
        while True:
            arm_response = read_command(ser)
            if(arm_response == 'y'):
                break
            else:
                time.sleep(0.5)
        arm_drone(master)
        while True:
            if is_armed(master):
                send_command(ser, ADDRESS, "Drone is armed!")
                break            
            else:
                send_command(ser, ADDRESS,"Drone is not armed")      
                time.sleep(1)
        time.sleep(5)
        disarm_drone(master)
except KeyboardInterrupt:  
    print("Can't connect")

 
ser.close()
