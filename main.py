from pymavlink import mavutil
import serial
import time
import sys
from connect_to_vehicle import connect_to_vehicle
from arm_drone import arm_drone, is_armed
from disarm_drone import disarm_drone
from lora import send_command, get_address, get_network, read_command
from takeoff import takeoff
from set_movment import fly_movment, fly_to_waypoint
from get_location import get_location
from if_number import is_number_float, is_number_int
from drone_menu import send_drone_flypath_menu
from land import land
serial_port = '/dev/ttyUSB1'
baud_rate = 115200  # Default baud rate for RYLR998
GC_Address = 1
altitude = 1.5 #defalut altitude

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

        arm_drone(master)
        time.sleep(2)
        while not is_armed(master):
            send_command(ser, ADDRESS, "INFO.Drone is not armed")  
            arm_drone(master)                   
            time.sleep(3)

        send_command(ser, ADDRESS, "INFO.Drone is armed!")        
        time.sleep(5)

        takeoff(master, altitude)

        while True:          
            drone_command = send_drone_flypath_menu(ser,GC_Address)
            while not is_number_int(drone_command):
                drone_command = send_drone_flypath_menu(ser,GC_Address)    
            match int(drone_command):
                    case 1:
                        current_distance = 0
                        start_lat, start_lon, start_alt = get_location(master)
                        send_command(ser, GC_Address, "INPUT.Enter Distance to Fly Forward")
                        distance = read_command(ser)

                        while not is_number_float(distance):
                            send_command(ser, GC_Address, "INPUT.Enter valid Distance value")
                            distance = read_command(ser)  
                        distance = float(distance)                    
                        while current_distance < distance:
                                fly_movment(master, current_distance, distance, start_lat, start_lon)
                        send_command(ser, GC_Address, "INFO.Has reach to the target distance")
                    case 7:
                        land()
                        break

                    case _:
                        send_command(ser, GC_Address,"INFO.Invalid input")        
        disarm_drone(master)

except KeyboardInterrupt:
    print("Can't connect")

ser.close()
