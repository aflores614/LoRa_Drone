from pymavlink import mavutil
import logging
import math
import time
from lidar_distance import get_distance
from get_location import get_location
from travel_distance import distance_travel



check_interval = 0.5

def fly_movment(master, vx, vy, vz, ALT, Safe_Dist, Travel_distance, Target_distance, Home_lat, Home_lon ):
   
    # vx value for foward
    # vy value for right
    # vz value for down negative value for up 
    # Bitmask to indicate to use Velocity only and Z position (positive is down) : 0b110111000011
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                                 int(0b110111000000), 
                                                                                 0, 0, 1.1, 
                                                                                 vx, vy , vz, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                 ))
    if(vx > 0):
       
        while Travel_distance <= Target_distance:
            print("waiting for distance")
            dist_front = get_distance()                
            print("Front Distance", dist_front)
            logging.info("Front Distance: %.2f meters" %dist_front) 
                
            if( dist_front > Safe_Dist ): 
                print("Safe to travel Forward") 
                logging.info("Safe to fly forward")
                Current_lat, Current_lon, Current_alt = get_location(master) 
                Travel_distance = distance_travel(Home_lat, Current_lat, Home_lon, Current_lon) 
                print("Current distance travel: ", Travel_distance)
                logging.info("Distance traveled: %.2f meters" % Travel_distance) 
                time.sleep(check_interval)                      
            else:
                print("Obstacle detected")
                break
        
                
    else: 
                      
        while Travel_distance <= Target_distance:
            dist_front = get_distance()
            if( dist_front < Safe_Dist ): 
                print("Flying Backward") 
                logging.info("Flying Backward")               
                Current_lat, Current_lon, Current_alt = get_location(master) 
                Travel_distance = distance_travel(Home_lat, Current_lat, Home_lon, Current_lon)
                print("Current distance travel: ", Travel_distance)
                logging.info("Distance traveled: %.2f meters" % Travel_distance)  
                time.sleep(check_interval)                
            else:       
                print("No Obstacle Detected")         
                break 
  
    return Travel_distance
       
   

         

    
    

def fly_to_postion(master, lat, lon, ALT):
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                                                                                 int(0b110111111000), 
                                                                                 int(lat*1e7), int(lon* 1e7), ALT, 
                                                                                 0, 0 , 0, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))
    tolerance=0.00001 # how close the drone needs to get to the target position before the loop breaks
    while True:
        current_lat, current_lon, current_alt = get_location(master)
        if(abs(abs(lat) - abs(current_alt)) < tolerance and
           abs(abs(lon) - abs(current_lon)) < tolerance ):
            print("Reach target position")
            break
        else:
            print("Enroute to target Position")
        time.sleep(0.1)
        
