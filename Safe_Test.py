from pymavlink import mavutil
import time
import logging
from lidar_distance import get_distance
from abort_mission import abort_mission
from travel_distance import distance_travel
from get_location import get_location
from set_movment import fly_to_postion

def saftey_test_1(master, Safe_Dist):
    print("Stafey Test 1")
    while True:
        dist_front = get_distance()
        logging.info("Sensor readings - Front: %.2f",
                            dist_front)
        
        print ("Measured front Distance = %.1f m" % dist_front)
        if(dist_front <= Safe_Dist):
            print("Object too close")
            logging.warning("Object too close")                        
        elif( dist_front > Safe_Dist ):
            print("Safe")
            logging.info("Safe distance, proceeding")
            break
        else: # not safe to continue
            print(" Not safe to fly abort mission")
            print("Safty Test 1 Fail")
            logging.error("Not safe to fly, aborting mission")
            abort_mission(master)
            

def saftey_test_2(master, Home_lat, Home_lon, Alt):
    print("Testing Movement")
    current_lat, current_lon, current_alt = get_location(master)
    print(current_lat, current_lon, current_alt)
    logging.info("TAKE OFF Position: %f, %f, %f" % (current_lat, current_lon, current_alt))
    distance_travel_home = distance_travel(Home_lat, current_lat, Home_lon, current_lon)
    print("It had travel", distance_travel_home ,"meters")  
    logging.info("It had traveled {} meters".format(distance_travel_home))
    if( distance_travel_home < 1.5):
        #fly_to_postion(master, Home_lat, Home_lon, current_alt)
        print("Safe Range")
    else:
        print("not safe to fight to home position")
        logging.info("Not safe to fight to home position ")                     
