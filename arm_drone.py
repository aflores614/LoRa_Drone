from pymavlink import mavutil
from connect_to_vehicle import connect_to_vehicle
import time
def arm_drone(master):
    # Send a command to arm the drone
    master.mav.command_long_send(
        master.target_system,                
        master.target_component,             
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  
        0,                                  
        1,                                   # param1 (1 to arm, 0 to disarm)
        21196, 0, 0, 0, 0, 0                     
    )
    time.sleep(5)

  
    
    
def is_armed(master):
    # Request the current system status
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
        0, 0, 0, 0, 0, 0
    )
    
    time.sleep(1)
    heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
    return heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0
 
if __name__ == "__main__":    
    master = connect_to_vehicle()
    while True:        
        arm_drone(master)
        if is_armed(master):
            break


