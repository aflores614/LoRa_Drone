from pymavlink import mavutil
from get_location import get_location
import math
def fly_forward(master, distance):
    lat,lon, alt = get_location(master) 
    Earth_R = 6378137.0

    offset_lat = distance / Earth_R
    offset_lon = distance / (Earth_R * math.cos(math.radians(lat)))

    New_lat = lat + (offset_lat * 180/ math.pi)
    New_lon = lon + (offset_lon * 180 / math.pi)

    print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")
    print(f"New Latitude: {New_lat}, New Longitude: {New_lon }, Altitude: {alt}")
    print("It's safe to continue (y/n)")
    safe = input().strip().lower()

    if(safe == 'y'):
        master.mav.command_long_send(
            master.target_system,  # target system
            master.target_component,  # target component
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # command
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4 (not used for this command)
            New_lat * 1e7,  # param 5: latitude in degrees * 1e7
            New_lon * 1e7,  # param 6: longitude in degrees * 1e7
            alt * 1000  # param 7: altitude in millimeters
        )
    else:
        print("Not safe")