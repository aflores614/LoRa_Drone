from pymavlink import mavutil
import logging
import math
import time
import signal
from lidar_distance import get_distance
from get_location import get_location
from travel_distance import distance_travel
from fly_forward import get_waypoint



check_interval = 0.5
vx = 1
class TimeoutException(Exception):
    pass
def handler(signum, frame):
    raise TimeoutException()

def fly_movment(master, Travel_distance, Target_distance, Home_lat, Home_lon):
   
    # vx value for foward
    # vy value for right
    # vz value for down negative value for up 
    # Bitmask to indicate to use Velocity only and Z position (positive is down) : 0b110111000011
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                                 int(0b110111111000 ), 
                                                                                 Target_distance, 0, 0, 
                                                                                 0, 0 , 0, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))
                                                                                
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print(msg)
    if (msg == True and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED):
        end_time = time.time() + 1
        #sending velocity commands, they should be re-sent every second   
        while Travel_distance <= Target_distance or time.time() < end_time:   
            Current_lat, Current_lon, Current_alt = get_location(master) 
            Travel_distance = distance_travel(Home_lat, Current_lat, Home_lon, Current_lon)
            print("Current distance travel: ", Travel_distance)
            logging.info("Distance traveled: %.2f meters" % Travel_distance) 
            time.sleep(check_interval)            
    

    
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
    
    while True:     
        current_lat, current_lon, current_alt = get_location(master)
        lat_error = abs(abs(lat) - abs(current_lat))
        lon_error = abs(abs(lon) - abs(current_lon))        

        print(current_alt)

        
        if(lat_error < tolerance and lon_error < tolerance ):
            print("Reach target position")
            logging.info("Reach Target Position")
            break
        else:
            print("Enroute to target Position")
            logging.info("Enroute to target Position")
        time.sleep(0.25)

def fly_hover(master, alt):   
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
        
    if(dir == 0):
        for i in range(num_waypoint): #clock-wise fly path
            lat =  waypoints_lat[i]
            lon =  waypoints_lon[i]
            fly_to_waypoint(master, lat, lon, altitude )
    else:
        for i in range(num_waypoint): #counterclock-wise fly path
            lat =  waypoints_lat[num_waypoint - i - 1]
            lon =  waypoints_lon[num_waypoint - i - 1]
            fly_to_waypoint(master, lat, lon, altitude )



        
