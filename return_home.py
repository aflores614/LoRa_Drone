from pymavlink import mavutil
def return_home(master):
    master.mav.command_long_send(
        master.target_system,  # target system
        master.target_component,  # target component
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # command
        0,  # confirmation
        0, 0, 0, 0, 0, 0, 0  # params (not used for this command)
    )
    print("Return to home command sent.")