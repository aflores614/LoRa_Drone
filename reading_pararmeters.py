from pymavlink import mavutil
import time
def connect_to_vehicle():
    try:
        print("Connecting to vehicle on /dev/ttyAMA0 with baud rate 57600...")
        master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print("Heartbeat received")
        return master
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None

def read(master):
    print ("Started")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1,
        1)
    while True:
        msg = master.recv_match(type = 'GLOBAL_POSITION_INT', blocking = True)
        if msg is not None:            
            relative_altitude = msg.relative_alt/ 1000.0
            print(f"Altitude (MSL): Relative altitude: {relative_altitude:.2f} meters")
        
        else:
            print("No message Received")
       
        
master = connect_to_vehicle()

if master:
    read(master)
else:
        print("Can't connect to vehicle")

    
