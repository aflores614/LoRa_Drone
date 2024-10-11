from datetime import datetime
import logging

def setup_log_file():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"/home/pi/Drone_Test_Logs/drone_LoRa_log_{timestamp}.log"
    logging.basicConfig(filename= log_filename, 
                            level=logging.INFO,
                            format='%(asctime)s - %(levelname)s - %(message)s',
                            filemode='w')  
    logging.info("Start")