from pymavlink import mavutil
import time
def arm_drone(master):
    # Send a command to arm the drone
    master.mav.command_long_send(
        master.target_system,                
        master.target_component,             
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  
        0,                                  
        1,                                   # param1 (1 to arm, 0 to disarm)
        0, 0, 0, 0, 0, 0                     
    )

    #ack = master.recv_match(type = 'Command_ACK', blocking = True)
    
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



