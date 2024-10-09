from lora import send_command, read_command
import serial
def send_drone_flypath_menu(ser,GC_Address):
    send_command(ser, GC_Address, "INFO.1 = FLY FORWARD")
    send_command(ser, GC_Address, "MENU.2 = WAYPOINT")
    send_command(ser, GC_Address, "MENU.3 = HOVER")
    send_command(ser, GC_Address, "MENU.4 = RETURN HOME")
    send_command(ser, GC_Address, "MENU.5 = CIRCLE")
    send_command(ser, GC_Address, "MENU.6 = RETURN HOME AND LAND")
    send_command(ser, GC_Address, "MENU.7 = LAND")
    send_command(ser, GC_Address, "MENU.8 = Change Altitude Value ")
    send_command(ser, GC_Address, "INPUT.9 = LoRa Com Test ")
    drone_command = read_command(ser)
    return drone_command
if __name__ == "__main__":
    serial_port = '/dev/ttyUSB0'
    baud_rate = 115200  # Default baud rate for RYLR998
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    send_drone_flypath_menu(ser,2)
