from pymavlink import mavutil
import serial
import time
import sys
from connect_to_vehicle import connect_to_vehicle
from arm_drone import arm_drone, is_armed
from disarm_drone import disarm_drone
from lora import send_command, get_address, get_network, read_command
from takeoff import takeoff
from set_movment import fly_movment, fly_to_waypoint, fly_hover,fly_circle
from get_location import get_location
from if_number import is_number_float, is_number_int
from drone_menu import send_drone_flypath_menu
from travel_distance import distance_travel
from land import land


serial_port = '/dev/ttyUSB0'
baud_rate = 115200  # Default baud rate for RYLR998
GC_Address = 2
altitude = 2.5 #defalut altitude

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

        while True: #ask to arm the drone
            send_command(ser, GC_Address, "INPUT.Arm Drone y/n: ")
            arm_response = read_command(ser)
            
            if arm_response == 'y':
                break
            elif arm_response == 'n':
                sys.exit()
            else:
                send_command(ser, GC_Address, "ERROR.Invalid input")      
        home_lat, home_lon, home_alt = get_location(master)
        arm_drone(master)
        arm = is_armed(master)
        
        while not arm: #drone is not arm
            send_command(ser, ADDRESS, "INFO.Drone is not armed")  
            arm_drone(master)           
            arm = is_armed(master)
            print(arm)

        send_command(ser, ADDRESS, "INFO.Drone is armed!")  
	      
        time.sleep(5)

        takeoff(master, altitude)

        while True:          
            drone_command = send_drone_flypath_menu(ser,GC_Address)
            while not is_number_int(drone_command):
                drone_command = send_drone_flypath_menu(ser,GC_Address)    
            match int(drone_command):
                    case 1: #fly foward in meters 
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
                                Current_lat, Current_lon, Current_alt = get_location(master)
                                current_distance = distance_travel(home_lat, Current_lat, home_lon, Current_lon)
                        send_command(ser, GC_Address, "INFO.Has reach to the target distance")
                    case 2: #fly to a waypoint
                        send_command(ser, GC_Address, "INPUT.Enter Latitude:  ")
                        waypoint_lat = read_command(ser)
                        while not is_number_float(waypoint_lat):       
                            send_command(ser, GC_Address, "INPUT.Enter Latitude:  ")
                            waypoint_lat = read_command(ser)
                        waypoint_lat = float(waypoint_lat)

                        send_command(ser, GC_Address, "INPUT.Enter Longitude:  ")
                        waypoint_lon = read_command(ser)
                        while not is_number_float(waypoint_lon):       
                            send_command(ser, GC_Address, "INPUT.Enter Longitude:  ")
                            waypoint_lon = read_command(ser)
                        waypoint_lon = float(waypoint_lon)

                        fly_to_waypoint(master, waypoint_lat, waypoint_lon, altitude )
                    case 3: #Hover 
                        send_command(ser, GC_Address, "INPUT.Enter atitude:  ")
                        ALT = read_command(ser)
                        while not is_number_float(ALT):
                            send_command(ser, GC_Address, "INPUT.Enter atitude:  ")
                            ALT = read_command(ser)
                        ALT = float(ALT)
                        fly_hover(master, ALT )
                    case 4: #return home
                        fly_to_waypoint(master, home_lat, home_lon, altitude )
                    case 5: #circle mode
                        send_command(ser, GC_Address, "INPUT.Enter Radius:  ")
                        Radius = read_command(ser)
                        while not is_number_float(Radius):
                            send_command(ser, GC_Address, "INPUT.Enter Radius:  ")
                            Radius = read_command(ser)
                        Radius = float(Radius)
                        fly_circle(master, Radius, 0) #clockwise
                    case 6: #return home and land
                        fly_to_waypoint(master, home_lat, home_lon, altitude )
                        time.sleep(5)
                        land()
                    case 7: #land and break
                        land()
                        break
                    case _: #error input
                        send_command(ser, GC_Address,"INFO.Invalid input")        
        disarm_drone(master)
        send_command(ser, ADDRESS, "INFO.Drone is disarmed!")  
	

except KeyboardInterrupt:
    print("Can't connect")
    sys.exit()

ser.close()

