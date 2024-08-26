from math import radians, cos, sin, asin, sqrt
import logging
def distance_travel(lat_1, lat_2, lon_1, lon_2):
    lon_1 = radians(lon_1)
    lon_2 = radians(lon_2)
    lat_1 = radians(lat_1)
    lat_2 = radians(lat_2)
      
    # Haversine formula 
    dlon = lon_2 - lon_1 
    dlat = lat_2 - lat_1
    a = sin(dlat / 2)**2 + cos(lat_1) * cos(lat_2) * sin(dlon / 2)**2
 
    c = 2 * asin(sqrt(a)) 
    
    # Radius of earth 6371 in kilometers. Use 3956 for miles

    r = 6371000 # radius of earth in meters
      

    return(round(c * r, 2))

 
if __name__ == "__main__":
    logging.basicConfig(filename='drone_log.log', 
                    level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    filemode='w')  # 'w' for overwrite, 'a' for append
    lat_1 = 53.32055555555556
    lat_2 = 53.32055555555556
    lon_1 = -1.7297222222222221
    lon_2 =  -1.7297
    #print("It had travel", distance_travel(lat_1, lat_2, lon_1, lon_2), "meters")
    distance = distance_travel(lat_1, lat_2, lon_1, lon_2)
    logging.info(f"Distance front: {distance} m")
