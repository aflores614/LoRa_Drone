from pymavlink import mavutil
import serial
import time
import sys
import logging
from connect_to_vehicle import connect_to_vehicle
from arm_drone import arm_drone, is_armed
from disarm_drone import disarm_drone
from lora import send_command, get_address, get_network, read_command,set_parameter
from takeoff import takeoff
from set_movment import fly_movment, fly_to_waypoint, increse_alt,fly_circle
from get_location import get_location
from if_number import is_number_float, is_number_int
from drone_menu import send_drone_flypath_menu
from travel_distance import distance_travel
from check_pre_arm import check_pre_arm
from land import land
from datetime import datetime
from Range_Test import test_lora_comm_range
from log_file import setup_log_file

setup_log_file()

serial_port = '/dev/ttyUSB0'
baud_rate = 115200  # Default baud rate for RYLR998
GC_Address = 2
altitude = 8 #defalut altitude




ser = serial.Serial(serial_port, baud_rate, timeout=1)

try:
    ADDRESS = get_address(ser)
    print("The ADDRESS is:", ADDRESS)
    
    network = get_network(ser)
    print("The network is:", network)
    
    set_parameter(ser,8,7,1,12)
 
    
    while True:  # Establish communication between Drone and Ground Control
        send_command(ser, GC_Address, "INPUT:Connection y/n")
        response = read_command(ser)
        logging.info("Connect to Drone?")
        print(response)
        
        if response == 'y':
            logging.info("Yes")
            break
        elif response == 'n':
            logging.info("No System OFF")
            sys.exit()
        else:
            send_command(ser, GC_Address, "ERROR:Invalid input")
    
    master = connect_to_vehicle()
    
    if master:
        send_command(ser, GC_Address, "INFO:Vehicle Connected")

        while True: #ask to arm the drone
            send_command(ser, GC_Address, "INPUT:Arm Drone y/n: ")
            arm_response = read_command(ser)
            logging.info("Arm Drone?")
            if arm_response == 'y':
                logging.info("Yes")
                break
            elif arm_response == 'n':
                logging.info("No System OFF")
                sys.exit()
            else:
                send_command(ser, GC_Address, "INFO:Invalid input")  
        if check_pre_arm(master):            
            home_lat, home_lon, home_alt = get_location(master)
            logging.info("Home Position: %f, %f, %f" % (home_lat, home_lon, home_alt))                        
            arm_drone(master)
            takeoff(master, altitude)           
            
        logging.info("Drone is Arm")
        send_command(ser, GC_Address, "INFO:Drone is armed!")        

   
        try:
            while True:          
                drone_command = send_drone_flypath_menu(ser,GC_Address)
                while not is_number_int(drone_command):
                    drone_command = send_drone_flypath_menu(ser,GC_Address)    
                match int(drone_command):
                    case 1: #fly foward in meters 
                        logging.info("Fly Movement")

                        send_command(ser, GC_Address, "INPUT:Enter X axis distance")
                        x = read_command(ser)
                        if (x == "cancel"):
                            break
                        while not is_number_float(x):
                            send_command(ser, GC_Address, "INPUT:Enter valid  x Distance value")
                            x = read_command(ser)  
                        x = float(x)         

                        send_command(ser, GC_Address, "INPUT:Enter y axis distance")
                        y = read_command(ser)
                        if (y == "cancel"):
                            break
                        while not is_number_float(y):
                            send_command(ser, GC_Address, "INPUT:Enter valid y Distance value")
                            y = read_command(ser)  
                        y = float(y)         

                        fly_movment(master, x, y)                                
                        send_command(ser, GC_Address, "ACK:Has reach to the target distance")
                    case 2: #fly to a waypoint
                        try:
                            logging.info("Fly to a Waypoint")
                            send_command(ser, GC_Address, "INPUT:Enter Latitude:  ")
                            waypoint_lat = read_command(ser)      
                            if (waypoint_lat == "cancel"):
                                break                  
                            while not is_number_float(waypoint_lat):       
                                send_command(ser, GC_Address, "INPUT:Enter Latitude:  ")
                                waypoint_lat = read_command(ser)
                            waypoint_lat = float(waypoint_lat)

                            send_command(ser, GC_Address, "INPUT:Enter Longitude:  ")
                            waypoint_lon = read_command(ser)
                            while not is_number_float(waypoint_lon):       
                                send_command(ser, GC_Address, "INPUT:Enter Longitude:  ")
                                waypoint_lon = read_command(ser)
                            waypoint_lon = float(waypoint_lon)

                            fly_to_waypoint(master, waypoint_lat, waypoint_lon, altitude )
                        except Exception as e:
                            logging.error("Fly to a Waypoint ERROR: %s", str(e), exc_info=True)
                            send_command(ser, GC_Address, "INFO:Fly to a Waypoint ERROR")
                    case 3: #Hover 
                        logging.info("Hover")
                        send_command(ser, GC_Address, "INPUT:Enter atitude:  ")
                        ALT = read_command(ser)
                        if (ALT == "cancel"):
                                break   
                        while not is_number_float(ALT):
                            send_command(ser, GC_Address, "INPUT:Enter atitude:  ")
                            ALT = read_command(ser)
                        ALT = float(ALT)
                        increse_alt(master, ALT )
                    case 4: #circle mode
                        logging.info("Circle Mode")
                        send_command(ser, GC_Address, "INPUT:Enter Radius:  ")
                        Radius = read_command(ser)
                        if (Radius == "cancel"):
                                break   
                        while not is_number_float(Radius):
                            send_command(ser, GC_Address, "INPUT:Enter Radius:  ")
                            Radius = read_command(ser)
                        Radius = float(Radius)
                        fly_circle(master, Radius,altitude, 0) #clockwise
                    case 5: #return home
                        logging.info("Return Home")
                        fly_to_waypoint(master, home_lat, home_lon, altitude )
                    
                    case 6: #return home and land
                        try:
                            logging.info("Return Home and Land")
                            fly_to_waypoint(master, home_lat, home_lon, altitude )
                            time.sleep(5)
                            land(master)
                        except Exception as e:
                            logging.error("Return Home ERROR: %s", str(e), exc_info=True)
                            send_command(ser, GC_Address, "INFO:Return Home ERROR")
                    case 7: #land and break
                        logging.info("Land")
                        land(master)
                        break
                    case 8: #change alt value
                        logging.info("Change Altitude Value")
                        send_command(ser, GC_Address, "INPUT:Enter New Altitude Value:  ")
                        altitude = read_command(ser)
                        if (altitude == "cancel"):
                            break
                        while not is_number_float(altitude):
                            send_command(ser, GC_Address, "INPUT:Enter Altitude:  ")
                            altitude = read_command(ser)
                        altitude = float(altitude)  
                        
                    case 9:
                        try:
                            logging.info("Testing Commication Range")
                            send_command(ser, GC_Address, "INPUT:Enter Distance Range:  ")
                            Target_distance = read_command(ser)
                            if (Target_distance == "cancel"):
                                break
                            while not is_number_float(Target_distance):
                                send_command(ser, GC_Address, "INPUT:Enter Distance Range:  ")
                                Target_distance = read_command(ser)
                            Target_distance = float(Target_distance)  
                        
                            pass_test = test_lora_comm_range(master, ser, GC_Address, Target_distance,altitude,home_lat, home_lon)                        
                            if (pass_test == False):
                                logging.info("LoRa Range has max out")
                                fly_to_waypoint(master, home_lat, home_lon, altitude )
                                land(master)
                                break
                        except Exception as e:
                            logging.error("TEST Commication RANGE ERROR: %s", str(e), exc_info=True)
                            send_command(ser, GC_Address, "INFO:TEST Commication RANGE ERROR")
                            land(master)
                            disarm_drone(master)
                            
                        
                    case _: #error input
                        send_command(ser, GC_Address,"INFO:Invalid input")        
            disarm_drone(master)
            send_command(ser, GC_Address, "INFO:Drone is disarmed!")  
        except Exception as e:
            logging.error("Drone Menu ERROR: %s", str(e), exc_info=True)
            send_command(ser, GC_Address, "INFO:Drone Menu ERROR")

except KeyboardInterrupt:
    logging.info("User On Rasberry pi Cancel")
    send_command(ser, GC_Address, "INFO:User On Rasberry pi Cancel")
finally:
    logging.info("Script Finish")
    send_command(ser, GC_Address, "INFO:FInish")
    ser.close()
    logging.shutdown() 
    sys.exit()

