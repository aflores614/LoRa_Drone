from pymavlink import mavutil
def disarm_drone(master):
        master.mav.command_long_send(
        master.target_system,                
        master.target_component,             
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  
        0,                                   
        0,                                   # param1 (0 to disarm, 1 to arm)
        0, 0, 0, 0, 0, 0                     
    )
        print("Disarming Drone")
