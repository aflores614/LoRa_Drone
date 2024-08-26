import serial
import time
from connect_to_vehicle import connect_to_vehicle

serial_port = '/dev/ttyUSB1'
baud_rate = 115200  # Default baud rate for RYLR998
target_distance = 5 # distance in meters
current_distance = 0 # The distance the drone has traveled so far
velocity_x = 1 # forward speed at 1 m/s
velocity_y = 0 # Right speed at 0.0 m/s
velocity_z = 0 # Down speed at 0.0 m/s
neg_velocity_x = -velocity_x # backward speed at 0.5 m/s
check_interval = 0.5 # The time interval between each check of the distance
ALT = 1.1 # fix altitude
Safe_Dist = 0.75 # safe distance

ser = serial.Serial(serial_port, baud_rate, timeout=1)

def send_command(command):
    ser.write((command + '\r\n').encode())
    time.sleep(0.5)  # Wait for the command to be processed
    response = ser.read_all().decode()
    return response
def read_command():
	while True:
	
		string = ser.readline()
		c =string.decode("utf-8")
		if( c != ""):
			if c.startswith('+RCV='):
				parts = c.split(',')
				sender_id = parts[0].split('=')[1]  
				message_length = parts[1]           
				message_payload = parts[2]          
				rssi = parts[3]                     
				snr = parts[4]                      
            
            # Print the separated message components				
				print(f"Message: {message_payload}")

# Example: Set the LoRa module address
response = send_command('AT+ADDRESS?')
print(response)
response = send_command('AT+NETWORKID?')
print(response)

master = connect_to_vehicle()
if master:
	send_command('AT+SEND=0,7,CONNECT')
else:
	send_command('AT+SEND=0,9,UNCONNECT')	

ser.close()