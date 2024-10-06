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
    send_command(ser, GC_Address, "TEST.Hello")
    rx = rx_test(ser)
    print(rx)
    return rx

def test_lora_comm_range(master, ser, GC_Address, Target_distance):  
    distance = 0  
    print( distance < Target_distance)
    while distance < Target_distance:        
        current_distance = 0
        distance_Interve = 1
        rx = tx_test(ser,GC_Address)
        if rx == True:
            start_lat, start_lon, start_alt = get_location(master)
            logging.info("Current Position: %f, %f, %f" % (start_lat, start_lon, start_alt))
            fly_movment(master, current_distance, distance_Interve, start_lat, start_lon)
        else:
            logging.info("Singal Loss")            
            return False
        distance = distance + 1
    return True
    
    


