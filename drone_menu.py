from lora import send_command, read_command

def send_drone_flypath_menu(ser,GC_Address):
    send_command(ser, GC_Address, "INFO.1 = FLY FORWARD")
    send_command(ser, GC_Address, "INFO.2 = WAYPOINT")
    send_command(ser, GC_Address, "INFO.3 = HOVER")
    send_command(ser, GC_Address, "INFO.4 = RETURN HOME")
    send_command(ser, GC_Address, "INFO.5 = CIRCLE")
    send_command(ser, GC_Address, "INFO.6 = RETURN HOME AND LAND")
    send_command(ser, GC_Address, "INFO.7 = LAND")
    send_command(ser, GC_Address, "INPUT.8 = Change Altitude Value ")
    drone_command = read_command(ser)
    return drone_command
