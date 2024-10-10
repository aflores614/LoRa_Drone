from pymavlink import mavutil
import time
from lora import send_command
import serial


GC_Address = 2
serial_port = '/dev/ttyUSB0'
baud_rate = 115200  # Default baud rate for RYLR998
ser = serial.Serial(serial_port, baud_rate, timeout=1)

def get_location(master,ser):    
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
                          
            return lat, lon, alt 
            
        else:
            send_command(ser, GC_Address, "Waiting..." )
            print("Waiting ...")
           

            
