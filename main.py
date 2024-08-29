import serial
import time
from connect_to_vehicle import connect_to_vehicle

serial_port = '/dev/ttyUSB1'
baud_rate = 115200  # Default baud rate for RYLR998

ser = serial.Serial(serial_port, baud_rate, timeout=1)

def send_command(ADDRESS, command):
    command_len = str(len(command))
    c = ('ALT+SEND=' + str(ADDRESS) + ',' + command_len + ',' + command + '\r\n')
    print(c)
    ser.write(('AT+SEND=' + str(ADDRESS) + ',' + command_len + ',' + command + '\r\n').encode())
   # time.sleep(0.5)  # Wait for the command to be processed
def get_address():
    ser.write(('AT+ADDRESS?\r\n').encode())  # Send the AT command to query the address
    time.sleep(0.5)  # Wait for the response
    response = ser.read_all().decode()  # Read and decode the response
    if response.startswith('+'):  # Check if the response is valid
        parts = response.split('=')  # Split the response at the '=' character
        print(parts)  # Print the parts for debugging
        return parts[1].strip()  # Return the address (after '='), stripped of any whitespace
def get_network():
    ser.write(('AT+NETWORKID?\r\n').encode())  # Send the AT command to query the address
    time.sleep(0.5)  # Wait for the response
    response = ser.read_all().decode()  # Read and decode the response
    if response.startswith('+'):  # Check if the response is valid
        parts = response.split('=')  # Split the response at the '=' character
        print(parts)  # Print the parts for debugging
        return parts[1].strip()  # Return the address (after '='), stripped of any whitespace

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
				print(f"RSSI: {rssi}")



ADDRESS = get_address()
print("The ADDRESS is: ", ADDRESS)
network = get_network()
print("The network is: ", network)
send_command(ADDRESS, "Andres")
 
ser.close()
