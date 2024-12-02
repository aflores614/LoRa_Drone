"""...........................................................
-- Engineer: Andres Flores
-- Description:  function to command a drone to ascend to a 
-- specified altitude using the MAVLink protocol. It sets the 
-- flight mode to GUIDED, sends a takeoff command
-- (MAV_CMD_NAV_TAKEOFF), and monitors altitude through 
-- GLOBAL_POSITION_INT messages until the target is reached 
-- or a timeout occurs.
................................................................"""
from pymavlink import mavutil
from set_mode import set_mode
import time
def takeoff(master,altitude):
    
    if not set_mode(master,'GUIDED'):
        print("Failed to set GUIDED mode. Check GPS signal, pre-arm checks, and parameters.")
        return

    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altitude)

    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1,
        1)
    start_time = time.time()
    timeout = 10
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            relative_altitude = msg.relative_alt/ 1000.0  # Altitude in meters
            if relative_altitude >= altitude - 0.1:  # Allow a small margin                
                break
            if time.time() - start_time > timeout:
                return   
        time.sleep(1)

 
