from pymavlink import mavutil
import sys
from connect_to_vehicle import connect_to_vehicle
from disarm_drone import disarm_drone
from land import land

def abort_mission(master):
    land(master)     
    disarm_drone(master)
    sys.exit()
    
if __name__ == "__main__":
    master = connect_to_vehicle
    abort_mission(master)
    

