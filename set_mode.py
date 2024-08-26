from pymavlink import mavutil
import time
# Function to change the mode
def set_mode(master,mode):
    if mode not in master.mode_mapping():
        print(f"Mode {mode} not supported")
        return False

    mode_id = master.mode_mapping()[mode]
    print(f"Setting mode: {mode}")
    master.set_mode(mode_id)

    # Confirm the mode change
    ack = False
    while not ack:
        msg = master.recv_match(type='COMMAND_ACK', blocking = True,timeout = 5)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Mode {mode} set successfully")
                ack = True
            else:
                print(f"Failed to set mode {mode}: {msg.result}")
                return False
        time.sleep(1)
    return True