from pymavlink import mavutil
import time
from lora import send_command
import serial
import sys
from land import land
import logging

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
        try:
            msg = master.recv_match(type = 'GPS_RAW_INT', blocking = True, timeout = 1)  
               
            if msg is not None:
                lat = msg.lat/1e7
                lon = msg.lon/1e7
                alt = msg.alt/1e3 
                if(lat == 0.0 or lon == 0.0 or alt == 0.0):
                    logging.info("INFO.GPS Bad Health EMERGENCY Landing")
                    send_command(ser, GC_Address, "INFO.GPS Bad Health")
                    send_command(ser, GC_Address, "INFO.EMERGENCY Landing")
                    land(master)                
                
                else:
                    return lat, lon, alt 
            
            elif(num_try < max_try):
                logging.info("INFO.WAITING FOR GPS")
                send_command(ser, GC_Address, "INFO.Waiting For Location" )
                num_try += 1
            else:
                logging.info("INFO.GPS NOT Responding EMERGENCY Landing")
                send_command(ser, GC_Address, "INFO.GPS NOT Responding")           
                send_command(ser, GC_Address, "INFO.EMERGENCY Landing")
        except Exception as e:
                logging.info("GPS NOT Responding ERROR")
                send_command(ser, GC_Address, "INFO.GPS NOT Responding ERROR")

            
