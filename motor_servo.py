import RPi.GPIO as GPIO
import time
import threading
from lidar_distance import get_distance

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pin for the servo signal
servo_pin = 26  # Change to the pin you're using

# Set up the GPIO pin for output
GPIO.setup(servo_pin, GPIO.OUT)

# Set up PWM on the servo pin, with a 50Hz frequency
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz is the typical frequency for servos
pwm.start(0)  # Start PWM with 0% duty cycle (off)

def set_servo_angle(angle):
  try:
    duty = 2 + (angle / 18)   # Convert angle to duty cycle
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.1)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(duty)
  except Exception as e:
        print(f"Error in set_servo_angle: {e}")


def lidar_motor(): 
 
 
    while True:
            #angle = 90  # Move servo to 90 degrees
            #set_servo_angle(angle)
        
            
            
            for angle in range(80, 100, 1):  # Increase angle from 0 to 180
                set_servo_angle(angle)
                time.sleep(0.1)  # Adjust the delay to control speed
              
        # Sweep from 180 to 0 degrees
           # for angle in range(100, 80, -5):  # Decrease angle from 180 to 0
            ##   time.sleep(1)  
              
         
            
            
            
            
#servo_thread = threading.Thread(target=lidar_motor)
#servo_thread.start()
#while True:
   
 #   dist_front = get_distance()
  #  print(dist_front)
