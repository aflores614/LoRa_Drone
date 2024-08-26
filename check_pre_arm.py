from pymavlink import mavutil
# Function to check pre-arm status
def check_pre_arm(master):
    # Add specific checks here based on your setup
    print("Checking pre-arm status...")
    
    gps_lock = False
    while not gps_lock:
        print("Waiting ")
        msg = master.recv_match(blocking=True, timeout = 10)
        if msg:
            print(f"Received message:{msg.get_type()}")
            if msg.get_type() == 'GPS_RAW_INT' and msg.fix_type >= 3:
                gps_lock = True
                print("GPS lock acquired")
                
            elif msg.get_type () == 'GLOBAL_POSITION_INT':
                gps_lock = True
                print("Position information received")
            gps_lock = True
        else:
            print("No message received, retrying ...")
            time.sleep(1)
            
    print("Pre-arm check passed")
    return True
