import serial
import time
import string
import pynmea2
import sys
import math as m
from pubnub.pnconfiguration import PNConfiguration
from pubnub.pubnub import PubNub
from pubnub.exceptions import PubNubException

#This script plots the latitude and longitude on the Google maps with the help of google maps api
pnChannel = "raspi-tracker";

pnconfig = PNConfiguration()
pnconfig.subscribe_key = "sub-c-6f54acc4-04dd-11ec-91fa-62dfa3a98328"
pnconfig.publish_key = "pub-c-4e90d659-26ab-4067-82bd-ac6a8652ab32"
pnconfig.ssl = False

pubnub = PubNub(pnconfig)
pubnub.subscribe().channels(pnChannel).execute()
lat=0
lon=0
alt=800
while True:
    port="/dev/ttyAMA0"
    ser=serial.Serial(port, baudrate=9600, timeout=0.5)
    dataout = pynmea2.NMEAStreamReader()
    newdata=ser.readline()
    if sys.version_info[0] == 3:
        newdata_GNRMC = newdata.decode("utf-8", "ignore")
        newdata_GPGGA = newdata.decode("utf-8", "ignore")
    if newdata_GNRMC[0:6] == "$GNRMC":
        newmsg = pynmea2.parse(newdata_GNRMC)
        la = newmsg.latitude
        ln = newmsg.longitude
        # alt = newmsg. altitude
        sp=newmsg.data[6]
        l = ['%s: %s' % (newmsg.fields[i][0], newmsg.data[i])
             for i in range(len(newmsg.fields))]
        print(l)
        print('Lat=',la,'Lon=',ln,'sp=',sp)
        if(newmsg.data[2]!=''):
            lat=la
            lng=ln

    if(lat and lon):
        try:
            envelope = pubnub.publish().channel(pnChannel).message({
                'lat1': lat,
                'lng1': lng,
                'lat2': lat+0.0001,
                'lng2': lng+0.0001,
            }).sync()
            print("publish timetoken: %d" % envelope.result.timetoken)
        except PubNubException as e:
            handle_exception(e)
