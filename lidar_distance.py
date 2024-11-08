import serial
import time
from threading import  Lock

ser = None
current_distance = None
distance_lock = Lock()

# Open serial port
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
except serial.SerialException as e:
    print(f"SerialException: {e}")
    exit()

# Function to send a command to set the sample rate
def set_sample_rate(rate):
    if rate < 1 or rate > 250:
        print("Sample rate must be between 1Hz and 250Hz.")
        return

    if ser.is_open:
        # Construct the command to set the sample rate
        command = bytearray([0x5A, 0x06, 0x03, rate, 0, 0, 0, 0])
        
        # Calculate the checksum
        checksum = sum(command) % 256
        command[-1] = checksum

        # Send the command
        ser.write(command)
    else:
        print("Serial port is not open.")

# Function to read and decode data from TF-Luna
def read_lidar_data():
    ser.flushInput()  # Clear any residual data
    data = ser.read(9)
    
    # Check if the data frame is complete
    if len(data) == 9:
        # Check for valid data frame starting with 0x59 0x59
        if data[0] == 0x59 and data[1] == 0x59:
            # Extract the distance value (2 bytes: data[2] and data[3])
            distance = data[2] + (data[3] << 8)
            return distance * 0.01  # Convert to meters
       
            
    else:
        print("Incomplete data frame received")
    return None

def avg_distance(num_samples):
    distances = []  # Initialize the list to store distances

    for _ in range(num_samples):
        dist = read_lidar_data()  # Call the function to get a distance measurement
        if dist is not None:  # Check if the measurement is valid
            distances.append(dist)
       

    if len(distances) == 0:
        return -1  # Return -1 if no valid distances were collected
    average_distance = sum(distances) / len(distances)
    
    return round(average_distance, 2)

def get_distance():
    global current_distance
    num_sample = 100
    set_sample_rate(250)
    while True:
        distance = avg_distance(num_sample) 
        if(distance < 0.25):  #lindar range is from 0.2 - 8.0 meters
            distance = 8
        with distance_lock:
                current_distance = distance

def get_current_distance():
    with distance_lock:
        return current_distance
    


if __name__ == "__main__":
    try:
        while True:
            
            distance = get_distance()
          
            if distance != -1:
                print(f"Distance: {distance} m")
            else:
                print("Failed to get valid distance.")
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")

