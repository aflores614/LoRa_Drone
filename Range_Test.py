from pymavlink import mavutil
import serial
import time
import sys
import logging
from lora import send_command
from get_location import get_location
from travel_distance import distance_travel


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

def test_lora_comm_range(master, ser, GC_Address, Target_distance, Home_lat, Home_lon):

    Travel_distance = 0
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                                 int(0b110111000000), 
                                                                                 Target_distance, 0, -0.5, 
                                                                                 0.5 , 0 , 0, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))
                                                                                
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if (msg == True and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED):
        end_time = time.time() + 1
        #sending velocity commands, they should be re-sent every second   
        while Travel_distance <= Target_distance or time.time() < end_time:   
            Current_lat, Current_lon, Current_alt = get_location(master) 
            logging.info(Current_lat, Current_lon)
            Travel_distance = distance_travel(Home_lat, Current_lat, Home_lon, Current_lon)
            string_travel_distance = "TEST." + str(Travel_distance) + " m"
            send_command(ser, GC_Address, string_travel_distance)
            response = rx_test(ser)
            if(response == False):
                break            
            time.sleep(check_interval) 


