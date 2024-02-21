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

PORT = "/dev/ttyUSB0"
BAUDRATE = 9600
#open serial port
device = XBeeDevice(PORT, BAUDRATE)
device.open()
print("The xbee is open")
lat = float(34.04285)
lon = float(-117.81194)
Targ_Location = dk.LocationGlobalRelative(lat, lon,30)  
print ("Coordinates saved, Connecting to vehicle")
connection_string = '/dev/ttyACM0'
vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)
print("connection is secure")

shortDate = datetime.datetime.today().strftime('%Y_%m_%d')
outputFile = "coordinate_output_" + shortDate +".txt"
f = open(outputFile, "a")

# working auto from past program
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = dk.VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    time.sleep(8)
    
    aTargetAltitude = 10

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


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

def goToLocation(latPosition, lonPosition, altPosition):
    location = dk.LocationGlobal(latPosition, lonPosition, altPosition)
    vehicle.simple_goto(location)
    return 1

def changeAltitude(change):
    location = dk.LocationGlobal(vehicle.location.global_relative_frame.lat, 
                              vehicle.location.global_relative_frame.lon, 
                              vehicle.location.global_relative_frame.alt + change)
    vehicle.simple_goto(location)
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

device.add_data_received_callback(callbackFunction())
while 1:
    current_loc = [vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon] #current coordinates of the drone
    print("sending data...")
    print(current_loc)
    device.send_data_broadcast(send_UAV_data_Xbee(
        "A", 
        vehicle.location.global_relative_frame.lat, 
        vehicle.location.global_relative_frame.lon, 
        vehicle.location.global_relative_frame.alt,
        vehicle.velocity,
        vehicle.airspeed, 
        vehicle.battery.voltage))
    
    f.write(str(int(time.time()*1000))+' ')
    f.write(str(vehicle.location.global_relative_frame.lat)+ ' ')
    f.write(str(vehicle.location.global_relative_frame.lon)+ '\n')
    time.sleep(0.2)

