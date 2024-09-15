from pymavlink import mavutil
from get_location import get_location
import math
def get_waypoint(master, distance, angle):
    lat,lon, alt = get_location(master)     
    Earth_R = 6378137.0

    lat = math.radians(lat)
    lon = math.radians(lon)
    offset = distance/Earth_R
    New_lat = lat + (offset * math.cos(angle))
    New_lon = lon + (offset * (math.sin(angle)/math.cos(lat)))

    New_lat = math.degrees(New_lat) 
    New_lon = math.degrees(New_lon) 

    #print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {10}")
    #print(f"New Latitude: {New_lat}, New Longitude: {New_lon }, Altitude: {10}")
    return New_lat, New_lon

if __name__ == "__main__":
    lat, lon =get_waypoint(1, 10, 0)