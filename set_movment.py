from pymavlink import mavutil
import logging
import math
import time
import serial
from lidar_distance import get_distance
from get_location import get_location
from travel_distance import distance_travel
from fly_forward import get_waypoint
from lora import send_command
from lidar_distance import  get_current_distance


serial_port = '/dev/ttyUSB1'
baud_rate = 115200  # Default baud rate for RYLR998
GC_Address = 2
ser = serial.Serial(serial_port, baud_rate, timeout=1)
check_interval = 0.1
speed = 10 # drone spreed 10 m/s
error = 0.5
safe_distance = 3

def STOP_FLY(master):   

    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                                 int(0b110111111000 ), 
                                                                                 0, 0, 0, 
                                                                                 0, 0 , 0, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))
                                                                                
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

def fly_movment(master, x, y):
   
    # vx value for foward
    # vy value for right
    # vz value for down negative value for up 
    # Bitmask to indicate to use Velocity only and Z position (positive is down) : 0b110111000011
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                                 int(0b110111111000 ), 
                                                                                 x, y, 0, 
                                                                                 0, 0 , 0, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))
                                                                                
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    
    if(x == 0):
        duration = ((abs(y) / speed  ) + error )   
    elif(y == 0):
        duration = ( (abs(x) / speed) + error )
    else:
        duration = (((abs(x) + abs(y)) / speed) + error)
        
    end_time = time.time() + duration + error
    
    while time.time() < end_time:
        try:
            distance = get_current_distance()
        except Exception as e:
            logging.info("Error trying to get current distance")
            send_command(ser, GC_Address,"INFO:Error trying to get current distance")
   
        if distance < safe_distance:
            STOP_FLY(master)
            send_command(ser, GC_Address,"INFO:Obstacle Detected")
            logging.info("Obstacle Detected")
            break
        
        

    
def fly_to_waypoint(master, lat, lon, ALT):
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                                                 int(0b110111111000), 
                                                                                 int(lat*1e7), int(lon* 1e7), ALT, 
                                                                                 0, 0 , 0, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))
    tolerance=0.00001 # how close the drone needs to get to the target position before the loop breaks
    start_time = time.time()  # Start time to track timeout duration
    timeout = 120 

    while True:
            try: 
                current_lat, current_lon, current_alt = get_location(master)
            except Exception as e:
                logging.info("Error trying to current location in fly_to_waypoint function")
                send_command(ser, GC_Address,"INFO:GET_LOCATION in fly point fail")
                print("ERROR TRYING TO GET CURRENT LOCATION")
            try:
                distance = get_current_distance()
            except Exception as e:
                logging.info("Error trying to get current distance")
                send_command(ser, GC_Address,"INFO:Error trying to get current distance")
                
                
            lat_error = abs(abs(lat) - abs(current_lat))
            lon_error = abs(abs(lon) - abs(current_lon))              
            if time.time() - start_time < timeout:
                if distance < safe_distance:
                    STOP_FLY(master)
                    send_command(ser, GC_Address,"INFO:Obstacle Detected")
                    break
                if(lat_error < tolerance and lon_error < tolerance ):
                    logging.info("Reach Target Position")
                    send_command(ser, GC_Address,"INFO:Reach Target Position")     
                    break
                else:
                    logging.info("Enroute to target Position")
            else:
                send_command(ser, GC_Address,"INFO:Fly Waypoint timeout") 
                logging.info("Waypoint Timeout")
                break              
           
            time.sleep(check_interval)


        

def increse_alt(master, alt):   
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                                 int(0b110111000000), 
                                                                                 0, 0, -alt, 
                                                                                 0, 0 , 0, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))
    master.mav.request_data_stream_send(master.target_system,
                                        master.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                                        1,
                                        1)
    start_time = time.time()
    timeout = 10
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            relative_altitude = msg.relative_alt/ 1000.0  # Altitude in meters
            print(f"Current altitude: {relative_altitude}")
            if relative_altitude >= alt - 0.1:  # Allow a small margin                
                print(f"Reached target altitude of {alt} meters")
                break
            if time.time() - start_time > timeout:
                print("Timeout reached. Unable to reach target altitude.")
                return   
        time.sleep(1)
def fly_circle(master,radius,altitude,dir):
    
    num_waypoint = 8
    waypoints_lat = []
    waypoints_lon = []
    angle = 0
    for i in range(num_waypoint):
        lat, lon = get_waypoint(master, radius, angle)
        waypoints_lat.append(lat)
        waypoints_lon.append(lon)
        angle = angle + (360/num_waypoint)
        logging.info("Circle Waypoints : %f, %f" % (lat, lon))
        message = "INFO:Calcuate CIRCLE Waypoint " + str(i+1) + "/" + str(num_waypoint)
        send_command (ser, GC_Address, message) 
    if(dir == 0):
        for i in range(num_waypoint): #clock-wise fly path
            lat =  waypoints_lat[i]
            lon =  waypoints_lon[i]
            fly_to_waypoint(master, lat, lon, altitude )
            logging.info("Point %f complete " % (i+1))
            message = "ACK:Circle Waypoint " + str(i+1) + "/" + str(num_waypoint) 
            send_command(ser, GC_Address, message)
    else:
        for i in range(num_waypoint): #counterclock-wise fly path
            lat =  waypoints_lat[num_waypoint - i - 1]
            lon =  waypoints_lon[num_waypoint - i - 1]
            fly_to_waypoint(master, lat, lon, altitude )
            logging.info("Point %f complete " % (i+1))
            message = "ACK:Circle Waypoint " + str(i+1) + "/" + str(num_waypoint) 
            send_command(ser, GC_Address, message)
   
            
