from pymavlink import mavutil
import serial
import time
import sys
import logging
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
from check_pre_arm import check_pre_arm
from land import land
from datetime import datetime
from Range_Test import test_lora_comm_range

serial_port = '/dev/ttyUSB0'
baud_rate = 115200  # Default baud rate for RYLR998
GC_Address = 2
altitude = 5 #defalut altitude

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
log_filename = f"/home/pi/Drone_Test_Logs/drone_LoRa_log_{timestamp}.log"
logging.basicConfig(filename= log_filename, 
                        level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s',
                        filemode='w')  
logging.info("Start")

ser = serial.Serial(serial_port, baud_rate, timeout=1)

try:
    ADDRESS = get_address(ser)
    print("The ADDRESS is:", ADDRESS)
    
    network = get_network(ser)
    print("The network is:", network)
    
    while True:  # Establish communication between Drone and Ground Control
        send_command(ser, GC_Address, "INPUT.Connection y/n")
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
            send_command(ser, GC_Address, "ERROR.Invalid input")
    
    master = connect_to_vehicle()
    
    if master:
        send_command(ser, GC_Address, "INFO.Vehicle Connected")

        while True: #ask to arm the drone
            send_command(ser, GC_Address, "INPUT.Arm Drone y/n: ")
            arm_response = read_command(ser)
            logging.info("Arm Drone?")
            if arm_response == 'y':
                logging.info("Yes")
                break
            elif arm_response == 'n':
                logging.info("No System OFF")
                sys.exit()
            else:
                send_command(ser, GC_Address, "INFO.Invalid input")  
        if check_pre_arm(master):            
            home_lat, home_lon, home_alt = get_location(master)
            logging.info("Home Position: %f, %f, %f" % (home_lat, home_lon, home_alt))
            
            arm_count = 0
            max_retries = 5 
            arm_drone(master)       
            while not is_armed(master):
              arm_count += 1
              send_command(ser, GC_Address, "INFO.Drone is not armed retrying...")
              logging.info("Drone not arm retrying")
              time.sleep(5)
              arm_drone(master) #Retry to arm the drone
                
              if arm_count == max_retries:
                  send_command(ser, GC_Address, "INFO.ARM Fail Power OFF")
                  logging.info("ARM Fail")
                  sys.exit(1)        
    
        else:
           sys.exit()     
            
        logging.info("Drone is Arm")
        send_command(ser, GC_Address, "INFO.Drone is armed!")  
	      
      
        takeoff(master, altitude)
        if is_armed(master):      
            send_command(ser, GC_Address, "INFO.Drone Ready!")  
            print("System armed")
        else:
            print("system fail")
            send_command(ser, GC_Address, "INFO.ARM Fail, Power OFF")
            sys.exit()
            
                

        while True:          
            drone_command = send_drone_flypath_menu(ser,GC_Address)
            while not is_number_int(drone_command):
                drone_command = send_drone_flypath_menu(ser,GC_Address)    
            match int(drone_command):
                    case 1: #fly foward in meters 
                        logging.info("Fly Foward")
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
                                logging.info("Current Position: %f, %f, %f" % (Current_lat, Current_lon, Current_alt))
                        send_command(ser, GC_Address, "ACK.Has reach to the target distance")
                    case 2: #fly to a waypoint
                        logging.info("Fly to a Waypoint")
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
                        logging.info("Hover")
                        send_command(ser, GC_Address, "INPUT.Enter atitude:  ")
                        ALT = read_command(ser)
                        while not is_number_float(ALT):
                            send_command(ser, GC_Address, "INPUT.Enter atitude:  ")
                            ALT = read_command(ser)
                        ALT = float(ALT)
                        fly_hover(master, ALT )
                    case 4: #return home
                        logging.info("Return Home")
                        fly_to_waypoint(master, home_lat, home_lon, altitude )
                    case 5: #circle mode
                        logging.info("Circle Mode")
                        send_command(ser, GC_Address, "INPUT.Enter Radius:  ")
                        Radius = read_command(ser)
                        while not is_number_float(Radius):
                            send_command(ser, GC_Address, "INPUT.Enter Radius:  ")
                            Radius = read_command(ser)
                        Radius = float(Radius)
                        fly_circle(master, Radius,altitude, 0) #clockwise
                    case 6: #return home and land
                        logging.info("Return Home and Land")
                        fly_to_waypoint(master, home_lat, home_lon, altitude )
                        time.sleep(5)
                        land(master)
                    case 7: #land and break
                        logging.info("Land")
                        land(master)
                        break
                    case 8: #change alt value
                        logging.info("Change Altitude Value")
                        send_command(ser, GC_Address, "INPUT.Enter New Altitude Value:  ")
                        altitude = read_command(ser)
                        while not is_number_float(altitude):
                            send_command(ser, GC_Address, "INPUT.Enter Altitude:  ")
                            altitude = read_command(ser)
                        altitude = float(altitude)  
                    case 9:
                        logging.info("Testing Commication Range")
                        send_command(ser, GC_Address, "INPUT.Enter Distance Range:  ")
                        Target_distance = read_command(ser)
                        while not is_number_float(Target_distance):
                            send_command(ser, GC_Address, "INPUT.Enter Distance Range:  ")
                            Target_distance = read_command(ser)
                        Target_distance = float(Target_distance)  
                        
                        pass_test = test_lora_comm_range(master, ser, GC_Address, Target_distance,altitude,home_lat, home_lon)                        
                        if (pass_test == False):
                            logging.info("LoRa Range has max out")
                            fly_to_waypoint(master, home_lat, home_lon, altitude )
                            land(master)
                            break
                        
                    case _: #error input
                        send_command(ser, GC_Address,"INFO.Invalid input")        
        disarm_drone(master)
        send_command(ser, ADDRESS, "INFO.Drone is disarmed!")  
	

except KeyboardInterrupt:
    print("Can't connect")
    sys.exit()
logging.info("Script Finish")
ser.close()

