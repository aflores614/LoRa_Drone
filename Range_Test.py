from pymavlink import mavutil
import serial
import time
import sys
import logging
from lora import send_command
from get_location import get_location
from travel_distance import distance_travel
from set_movment import fly_movment


check_interval = 0.5

def rx_test(ser):
    end_time = time.time() + 2
    while time.time() < end_time: #5 seconds wait time
        string = ser.readline()
        c = string.decode("utf-8")
        if(c != ""):
            if c.startswith('+RCV='):
                parts = c.split(',')
                sender_id = parts[0].split('=')[1]  
                message_length = parts[1]           
                message_payload = parts[2]          
                rssi = parts[3]                     
                snr = parts[4]   
                logging.info("Signal Strength: " + rssi)
                
                return True
                
    return False

def tx_test(ser, GC_Address):
    send_command(ser, GC_Address, "TEST.LoRa_CT ")
    rx = rx_test(ser)
    print(rx)
    return rx

def test_lora_comm_range(master, ser, GC_Address, Target_distance, altitude):  
    distance = 0  
    i = 0
    num_waypoint = round(Target_distance/2)
    waypoints_lat = []
    waypoints_lon = []
    angle = 0
    intervals = Target_distance/num_waypoint
    for n in range(num_waypoint):
        lat, lon = get_waypoint(master, intervals, angle)
        waypoints_lat.append(lat)
        waypoints_lon.append(lon)
              

    while distance < Target_distance:        
        rx = tx_test(ser,GC_Address)
        if rx  and i < num_waypoint:
            lat =  waypoints_lat[i]
            lon =  waypoints_lon[i]
            start_lat, start_lon, start_alt = get_location(master)
            logging.info("Current Position: %f, %f, %f" % (start_lat, start_lon, start_alt))
            fly_to_waypoint(master, lat, lon, altitude )
            i += 1
            
        else:
            logging.info("Singal Loss")            
            return False
        distance += intervals
    return True
    
    


