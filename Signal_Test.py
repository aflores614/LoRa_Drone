from pymavlink import mavutil
import time
import serial
import sys
from connect_to_vehicle import connect_to_vehicle
from lora import send_command
from set_movment import  fly_to_waypoint
from land import land
from get_location import get_location

def rx_test(ser):
    end_time = time.time() + 60
    while time.time() < end_time: #10 seconds wait time
        string = ser.readline()
        c = string.decode("utf-8")
        if(c == ""):
            return True                     
    return False

def tx_test(ser, GC_Address):
    send_command(ser, GC_Address, "SIGNAL: ")
    rx = rx_test(ser)
    return rx


def signal_connection(master, ser, home_lat, home_lon, GC_Address, altitude):
    signal_loss = 0
    while True:
        signal_connect = tx_test(ser, GC_Address)
        if(signal_connect != True):
            signal_loss == signal_connect + 1
            if ( signal_loss == 3):
                fly_to_waypoint(master, home_lat, home_lon, altitude )
                land(master)
                break
        else:
            signal_loss = 0
        time.sleep(1)
        

if __name__ == "__main__":
    serial_port = '/dev/ttyUSB1'
    baud_rate = 115200  # Default baud rate for RYLR998
    GC_Address = 2
    altitude = 2 #defalut altitude
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    master = connect_to_vehicle()
    home_lat, home_lon, home_alt = get_location(master)
    signal_connection(master, ser, home_lat, home_lon, GC_Address, altitude)
