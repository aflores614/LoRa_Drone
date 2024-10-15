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
    end_time = time.time() + 10
    while time.time() < end_time: #10 seconds wait time
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
    send_command(ser, GC_Address, "TEST:LoRa_CT ")
    rx = rx_test(ser)
    print(rx)
    return rx

def test_lora_comm_range(master, ser, GC_Address, Target_distance, altitude, home_lat, home_lon):  
    distance = 0  
    i = 0
    num_waypoint = 4 
    waypoints_lat = []
    waypoints_lon = []
    angle = 0
    signal_not_response = 0
    intervals = Target_distance/num_waypoint
    print(num_waypoint, intervals) 

    logging.info("Total Distance for LoRa Comm Test: %f" % Target_distance)
    for n in range(num_waypoint):
        print("Point " , n)
        lat, lon = get_waypoint(master, (n+1)*intervals, angle)
        waypoints_lat.append(lat)
        waypoints_lon.append(lon)        
        logging.info("Waypoints : %f, %f" % (lat, lon))
        message = "INFO:Calcuate Waypoint " + str(n+1) + "/" + str(num_waypoint)
        send_command (ser, GC_Address, message) 
    send_command (ser, GC_Address, "INFO:Start Test") 
    while distance < Target_distance:        
        rx = tx_test(ser,GC_Address)
        if rx: 
            if i < num_waypoint:
                lat =  waypoints_lat[i]
                lon =  waypoints_lon[i]            
                fly_to_waypoint(master, lat, lon, altitude)            
                logging.info("Point %f complete " % (i+1))
                message = "ACK:Waypoint " + str(i+1) + "/" + str(num_waypoint) 
                send_command(ser, GC_Address, message)            
                i += 1
            else:
                logging.info("All waypoints complete")
                send_command(ser, GC_Address, "INFO:ALL Waypoint Complete")
                break
            
        else:
            logging.info("Singal Loss") 
            signal_not_response += 1
            if(signal_not_response == 5):           
                return False
            time.sleep(1)
        distance += intervals

    send_command(ser, GC_Address, "ACK:Flying back Home")   
    fly_to_waypoint(master, home_lat, home_lon, altitude )

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
    alt = 5
    master = connect_to_vehicle()
    send_command(ser, 2, "INFO:Test")
    home_lat, home_lon, home_alt = get_location(master) 
    print(home_lat, home_lon, home_alt)
    
    test = test_lora_comm_range(master, ser, 2, 100, alt, home_lat, home_lon)

