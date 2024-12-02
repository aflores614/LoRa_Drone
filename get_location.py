"""...........................................................
-- Engineer: Andres Flores
-- Description:cript retrieves the GPS location of a drone 
-- using MAVLink commands while monitoring the health of GPS data. 
-- If valid GPS data is unavailable after multiple attempts, it 
-- initiates an emergency landing and logs the issue, ensuring 
-- safety during operation also calculate new coordinate waypoints
................................................................"""
from pymavlink import mavutil
from LoRa_Commands import send_command
import serial
from land import land
import logging
import math

num_try = 0
max_try = 10

GC_Address = 2
serial_port = '/dev/ttyUSB0'
baud_rate = 115200  # Default baud rate for RYLR998
ser = serial.Serial(serial_port, baud_rate, timeout=1)

def get_location(master):    
    master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1000000,
            0,0,0,0,0,0
    )
    while True:
        
        msg = master.recv_match(type = 'GPS_RAW_INT', blocking = True, timeout = 1)  
               
        if msg is not None:
            lat = msg.lat/1e7
            lon = msg.lon/1e7
            alt = msg.alt/1e3 
            if(lat == 0.0 or lon == 0.0 or alt == 0.0):
                    logging.info("INFO:GPS Bad Health EMERGENCY Landing")
                    send_command(ser, GC_Address, "INFO:GPS Bad Health")
                    send_command(ser, GC_Address, "INFO:EMERGENCY Landing")
                    land(master)                
                
            else:
                    return lat, lon, alt 
            
        elif(num_try < max_try):
            logging.info("INFO.WAITING FOR GPS")
            send_command(ser, GC_Address, "INFO:Waiting For Location" )
            num_try += 1
        else:
            logging.info("INFO:GPS NOT Responding EMERGENCY Landing")
            send_command(ser, GC_Address, "INFO:GPS NOT Responding")           
            send_command(ser, GC_Address, "INFO:EMERGENCY Landing")

def get_waypoint(master, distance, angle):
    lat,lon, alt = get_location(master)     
    Earth_R = 6378137.0

    lat = math.radians(lat)
    lon = math.radians(lon)
    offset = distance/Earth_R
    New_lat = lat + (offset * math.cos(angle))
    New_lon = lon + (offset * (math.sin(angle)/math.cos(lat)))

    New_lat = math.degrees(New_lat) 
    New_lon = math.degrees(New_lon) 

    return New_lat, New_lon

            
