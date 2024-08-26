from pymavlink import mavutil
import time
from get_location import get_location
from connect_to_vehicle import connect_to_vehicle

master = connect_to_vehicle()
Home_lat, Home_lon, Home_alt = get_location(master)
print("Home postion is set")
print(Home_lat, Home_lon, Home_alt)