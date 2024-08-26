import RPi.GPIO as GPIO
import time
import sys 
#set GPIO Pins
GPIO_TRIGGER_E = 23
GPIO_ECHO_E = 24

GPIO_TRIGGER_W = 17
GPIO_ECHO_W = 27

GPIO_TRIGGER_S = 5
GPIO_ECHO_S = 6

GPIO_TRIGGER_N = 13
GPIO_ECHO_N = 26

#GPIO Mode (BOARD / BCM) 
GPIO.setmode(GPIO.BCM)

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER_N, GPIO.OUT)
GPIO.setup(GPIO_ECHO_N, GPIO.IN)

GPIO.setup(GPIO_TRIGGER_S, GPIO.OUT)
GPIO.setup(GPIO_ECHO_S, GPIO.IN)

GPIO.setup(GPIO_TRIGGER_E, GPIO.OUT)
GPIO.setup(GPIO_ECHO_E, GPIO.IN)

GPIO.setup(GPIO_TRIGGER_W, GPIO.OUT)
GPIO.setup(GPIO_ECHO_W, GPIO.IN)
 
def distance(GPIO_TRIGGER, GPIO_ECHO):

   
    GPIO.output(GPIO_TRIGGER, True)
   
    # set Trigger after 0.01ms to LOW    
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
        
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
        
        
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2 then 100 to get meter
    distance = ((TimeElapsed * 34300) / 2) / 100
    
    if distance > 4.5 or distance < 0.02:
        return 4.5
 
    return distance 
 
def avg_distance(num_samples, GPIO_TRIGGER, GPIO_ECHO):
    distances = []
    for _ in range(num_samples):
        dist = distance(GPIO_TRIGGER, GPIO_ECHO)
        if dist != -1:
            distances.append(dist)
    if len(distances) == 0:
         return -1
    return round(sorted(distances)[len(distances)//2], 2)
    


def get_distance():
    num_sample = 30
    #dist_E = avg_distance(num_sample,GPIO_TRIGGER_E, GPIO_ECHO_E ) 
    #dist_W = avg_distance(num_sample,GPIO_TRIGGER_W, GPIO_ECHO_W )   
    #dist_S = avg_distance(num_sample,GPIO_TRIGGER_S, GPIO_ECHO_S)   
    dist_N = avg_distance(num_sample,GPIO_TRIGGER_N, GPIO_ECHO_N)   
    try:
        return  dist_N
    except Exception as e:
        return None, None, None, None
        
if __name__ == "__main__":
    while True:
        start_time = time.time()
        dist_N = get_distance() 
        end_time = time.time()
        total_time = end_time - start_time
        print(total_time)
       
        if(dist_N < 1.0):
                print("Obstacle Dectect")
                print("Measured North Distance = %.2f m" % dist_N)
                print(dist_N)
        else:
            #print("Measured East Distance = %.2f m" % dist_E)
            #print("Measured West Distance = %.2f m" % dist_W)            
            #print("Measured South Distance = %.2f m" % dist_S)
            print("Measured North Distance = %.2f m" % dist_N)
         


                    
