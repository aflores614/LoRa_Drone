import serial
import time
import logging



def send_command(ser, ADDRESS, command):
    command_len = str(len(command))
    c = ('AT+SEND=' + str(ADDRESS) + ',' + command_len + ',' + command + '\r\n')   
    ser.write(('AT+SEND=' + str(ADDRESS) + ',' + command_len + ',' + command + '\r\n').encode())
    time.sleep(0.5)  # Wait for the command to be processed
def get_address(ser):
    ser.write(('AT+ADDRESS?\r\n').encode())  # Send the AT command to query the address
    time.sleep(0.5)  # Wait for the response
    response = ser.read_all().decode()  # Read and decode the response
    if response.startswith('+'):  # Check if the response is valid
        parts = response.split('=')  # Split the response at the '=' character

        return parts[1].strip()  # Return the address (after '='), stripped of any whitespace
def get_network(ser):
    ser.write(('AT+NETWORKID?\r\n').encode())  # Send the AT command to query the address
    time.sleep(0.5)  # Wait for the response
    response = ser.read_all().decode()  # Read and decode the response
    if response.startswith('+'):  # Check if the response is valid
        parts = response.split('=')  # Split the response at the '=' character
        
        return parts[1].strip()  # Return the address (after '='), stripped of any whitespace

def read_command(ser):
    end_time = time.time() + 10
    while time.time() < end_time: #10 seconds wait time
        string = ser.readline()
        c = string.decode("utf-8")
        if(c != ""):
            if c.startswith('+RCV='):
                parts = c.split(',')
                sender_id = parts[0].split('=')[1]  
                message_length = parts[1]           
                message_payload = parts[2]          
                rssi = parts[3]                     
                snr = parts[4]   
                logging.info("Signal Strength: " + rssi)
                return message_payload
                
    return "No input"
