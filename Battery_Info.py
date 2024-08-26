from pymavlink import mavutil
from connect_to_vehicle import connect_to_vehicle

def Battery_Volatage(master):
    msg = master.recv_match(type='BATTERY_STATUS')
    if msg:
      # The `voltages` field is a list of voltages in millivolts (mV)
        voltage = msg.voltages[0] / 1000.0  # Convert from mV to Volts for the first battery
        return voltage
if __name__ == "__main__":
    master = connect_to_vehicle()
    Battery_voltage = Battery_Volatage(master)
    print("Drone  battery is at ", Battery_voltage,"V")
