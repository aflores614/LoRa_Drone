from pymavlink import mavutil
import serial
import time
import sys
import logging
from connect_to_vehicle import connect_to_vehicle
from lora import send_command
from get_location import get_location
from travel_distance import distance_travel
from set_movment import fly_movment, fly_to_waypoint
from fly_forward import get_waypoint
from datetime import datetime

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
    num_waypoint = 4 
    waypoints_lat = []
    waypoints_lon = []
    angle = 0
    intervals = Target_distance/num_waypoint
    print(num_waypoint, intervals) 

    logging.info("Total Distance for LoRa Comm Test: %f" % Target_distance)
    for n in range(num_waypoint):
        lat, lon = get_waypoint(master, n*intervals, angle)
        waypoints_lat.append(lat)
        waypoints_lon.append(lon)        
        logging.info("Waypoints : %f, %f" % (lat, lon))
    
    while distance < Target_distance:        
        rx = tx_test(ser,GC_Address)
        if rx  and i < num_waypoint:
            lat =  waypoints_lat[i]
            lon =  waypoints_lon[i]
            current_lat, current_lon, current_alt = get_location(master)
            logging.info("Point %f complete " % (i+1))
            logging.info("Current Position: %f, %f, %f" % (current_lat, current_lon, current_alt))
            #fly_to_waypoint(master, lat, lon, altitude )
            
            i += 1
            
        else:
            logging.info("Singal Loss")            
            return False
        distance += intervals
    logging.info("Test Past of %f meters" % distance)
    return True
    
    
if __name__ == "__main__":
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"/home/pi/random_logs/drone_LoRa_log_{timestamp}.log"
    logging.basicConfig(filename= log_filename, 
                        level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s',
                        filemode='w')  
    logging.info("Start")
    
    serial_port = '/dev/ttyUSB0'
    baud_rate = 115200  # Default baud rate for RYLR998
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    master = connect_to_vehicle()
    send_command(ser, 2, "INFO.Test")
    test = test_lora_comm_range(master, ser, 2, 300, 1.5)
