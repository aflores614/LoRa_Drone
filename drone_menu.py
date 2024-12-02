"""...........................................................
-- Engineer: Andres Flores
-- Description: sends a menu of flight path options to a drone 
-- via LoRa communication, allowing the operator to choose
................................................................"""
from LoRa_Commands import send_command, read_command
import serial
def send_drone_flypath_menu(ser,GC_Address):
    send_command(ser, GC_Address, "INFO:1 = FLY Movement")
    send_command(ser, GC_Address, "MENU:2 = WAYPOINT")
    send_command(ser, GC_Address, "MENU:3 = HOVER")
    send_command(ser, GC_Address, "MENU:4 = Circle")
    send_command(ser, GC_Address, "MENU:5 = Return Home")
    send_command(ser, GC_Address, "MENU:6 = RETURN HOME AND LAND")
    send_command(ser, GC_Address, "MENU:7 = LAND")
    send_command(ser, GC_Address, "INPUT:8 = LoRa Com Test ")
    drone_command = read_command(ser)
    return drone_command
if __name__ == "__main__":
    serial_port = '/dev/ttyUSB0'
    baud_rate = 115200  # Default baud rate for RYLR998
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    send_drone_flypath_menu(ser,2)
