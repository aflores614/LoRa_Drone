from pymavlink import mavutil
from set_mode import set_mode 
import time
#Function to land 
def land(master):
     if not set_mode(master,'LAND'):
        print("Failed to set LAND mode. Check GPS signal, pre-arm checks, and parameters.")
        return
     print("Landing Start")
     
     master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,0, 0, 0, 0, 0, 0,0)
     
     master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1,
        1)
     while True:        
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            relative_altitude = msg.relative_alt/ 1000.0  # Altitude in meters
            print(f"Current altitude: {relative_altitude}")
            if relative_altitude < 0.1:  # Allow a small margin                
                print("Drone has land ")
                break   
        time.sleep(1)  
