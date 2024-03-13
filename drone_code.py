from serial import Serial
import dronekit as dk           
from pymavlink import mavutil
import time 
from time import strftime
import datetime
from xbee import XBee
from digi.xbee.devices import XBeeDevice
import serial

print(serial.__file__)


shortDate = datetime.datetime.today().strftime('%Y_%m_%d')
outputFile = "coordinate_output_" + shortDate +".txt" # initializing an output file for logs
f = open(outputFile, "a")

''' Prompts user to pick flight mode'''
def get_flight_mode():
    print("Select flight mode:\n[1] - Autonomous\n[2] - Manual\n")
    mode = input()
    while mode != '1' and mode != '2':
        print("Invalid input. Try again")
        print("Select flight mode:\n[1] - Autonomous\n[2] - Manual\n")
        mode = input()
    if mode == '1':
        xbeeDevice.send_data_broadcast("Detection drone flight mode set as autonomous")
        return 1
    elif mode =='2':
        xbeeDevice.send_data_broadcast("Detection drone flight mode set as manual")
        return 2


# working auto from past program
''' Autonomous takeoff function from previous code'''
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not pixhawkVehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    pixhawkVehicle.mode = dk.VehicleMode("GUIDED")
    pixhawkVehicle.armed = True
    while not pixhawkVehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    time.sleep(8)
    
    aTargetAltitude = 10

    print("Taking off!")
    pixhawkVehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    while True:
        print(" Altitude: ", pixhawkVehicle.location.global_relative_frame.alt)
        if pixhawkVehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


''' Sends information to Xbee'''
def send_UAV_data_Xbee(ICAO, pos_lat, pos_lon, pos_alt_rel,velocity,airspeed,battery):
    #print("In send ADSB funtion\n")
    msg = "ICAO: " + ICAO + '\n'
    msg += "Lattitude: " + str(pos_lat) + '\n'
    msg += "Longitude: " + str(pos_lon) + '\n'
    msg += "Altitude: " + str(pos_alt_rel) + '\n'
    msg += "Velocity: " + str(velocity) + '\n'
    msg += "Airspeed: " + str(round(airspeed,5)) + '\n'
    #make sure that this line works; changed from vehicle.battery.voltage to battery parameter
    msg += "Battery Level: " + str(battery) + '\n'
    print(msg)
    return msg

''' Takes in location data and attempts to go there'''
def goToLocation(latPosition, lonPosition, altPosition):
    location = dk.LocationGlobal(latPosition, lonPosition, altPosition)
    pixhawkVehicle.simple_goto(location)
    return 1

''' Takes in change in altitude and makes adjustment'''
def changeAltitude(change):
    location = dk.LocationGlobal(pixhawkVehicle.location.global_relative_frame.lat, 
                              pixhawkVehicle.location.global_relative_frame.lon, 
                              pixhawkVehicle.location.global_relative_frame.alt + change)
    pixhawkVehicle.simple_goto(location)
    return 1

# the data that we get here is used to command the drone to do someting
# we are going to get a string in and fro there we will parse the string into simple parts
# this can be done with a key of letters to represent what function or command we are going to have the drone do
# with the amount of functions we have right now: alititude and go to a certain location; we should be fine
# then after that letter will contain the important infromation such as coordinates to a new location
# LEGEND/KEY/TABLE: a: go to altitude; l = go to location
# XBeeMessage is the variable that the data is going to be coming in
def callbackFunction(XBeeMessage):
    # this should hopefully work? if not try some stuff with XBeeMessage already existing?
    stringData = XBeeMessage.decode('utf-8')
    print(stringData)
    commandCharacter = XBeeMessage[0]
    print(commandCharacter)
    # if it is altitude, the only other characters in the string will be the increase or decrease of altitude 
    # the input for the change in altitude command:
    # first letter: a
    # second letter can be i or d to signify either an increase or decrease in altitude
    # the next digits will be the whole integer representation of how many meters it will change in the z direction
    if commandCharacter == 'a':
        altitudeChange = int(stringData[2:len(stringData)])
        if stringData[1] == 'd':
            altitudeChange = 0 - altitudeChange
        changeAltitude(altitudeChange)
    # Format:
    # first character: l
    # first batch of 10 characters: lat (2 integers and 7 decimal)
        # comma
    # next batch of 11 characters: lon (3 integers and 7 decimal)
        # comma
    # last batch of characters: altitude (can change)
    elif commandCharacter == 'l':
        goToLocation(int(stringData[1:11]), int(stringData[12:23]), int(stringData[24:len(stringData)]))
    return 1

def detect_while_spanning_area():
    # TODO: Check if Fire Scanning is active

    print("Enter the coordinates of the destination.")
    try:
        destLat = float(input("Enter the latitude of the destination: "))  
        destLon = float(input("Enter the longitude of the destination: "))
    except ValueError:
        print("Invalid input.")
        return

    print("Destination Coordinates Set: " + str(destLat) + ", " + str(destLon))
    xbeeDevice.send_data_broadcast("Destination Coordinates Set: " + str(destLat) + ", " + str(destLon))
    return


''' This section takes care of connections (xbee, pixhawk, etc) '''

PORT = "/dev/ttyUSB0" # port for xbee
BAUDRATE = 9600 # BAUDRATE for xbee 
''' Probably need to change the baud rate to 115200'''
#open serial port
xbeeDevice = XBeeDevice(PORT, BAUDRATE) # connect the device
xbeeDevice.open()
print("The xbee is open")

lat = float(34.04285)
lon = float(-117.81194)
Targ_Location = dk.LocationGlobalRelative(lat, lon,30)
print ("Coordinates saved, Connecting to vehicle")
connection_string = '/dev/ttyACM0' # connection string for pixhawk
pixhawkVehicle = dk.connect(connection_string, wait_ready=True, baud=115200) # connecting to pixhawk
print("connection is secure")

''' End of initialization section '''


xbeeDevice.add_data_received_callback(callbackFunction()) # add the callback function

detect_while_spanning_area()



while 1:
    current_loc = [pixhawkVehicle.location.global_relative_frame.lat, pixhawkVehicle.location.global_relative_frame.lon] #current coordinates of the drone
    print("sending data...")
    print(current_loc)
    xbeeDevice.send_data_broadcast(send_UAV_data_Xbee(
        "A", 
        pixhawkVehicle.location.global_relative_frame.lat, 
        pixhawkVehicle.location.global_relative_frame.lon, 
        pixhawkVehicle.location.global_relative_frame.alt,
        pixhawkVehicle.velocity,
        pixhawkVehicle.airspeed, 
        pixhawkVehicle.battery.voltage))
    
    f.write(str(int(time.time()*1000))+' ')
    f.write(str(pixhawkVehicle.location.global_relative_frame.lat)+ ' ')
    f.write(str(pixhawkVehicle.location.global_relative_frame.lon)+ '\n')
    time.sleep(0.2)

