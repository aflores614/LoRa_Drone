import serial
import time


serial_port = 'COM11'
baud_rate = 115200  # Default baud rate for RYLR998

ser = serial.Serial(serial_port, baud_rate, timeout=1)

def send_command(command):
    ser.write((command + '\r\n').encode())
    time.sleep(0.5)  # Wait for the command to be processed
    response = ser.read_all().decode()
    return response

# Example: Set the LoRa module address
response = send_command('AT+ADDRESS?')
print(response)
response = send_command('AT+NETWORKID?')
print(response)

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
			
			
	

# Example: Send a message
#response = send_command('AT+SEND=2,Hello')
#print(response)

# Close the serial connection
ser.close()