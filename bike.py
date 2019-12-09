import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import RPi.GPIO as GPIO
import serial
import os, time, sys
import urllib2
from time import sleep
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import picamera
import io
import numpy as np
import cv2


sensor = 21
switch = 20
motor = 16

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(sensor,GPIO.IN,GPIO.PUD_UP)
GPIO.setup(switch, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(motor, GPIO.OUT)

accelerometer_x = 0
accelerometer_y = 0
accelerometer_z = 0
temperature = 0
alcohole = 0
longitude = 0
latitude = 0

dist_meas = 0.00
km_per_hour = 0
rpm = 0
elapse = 0
pulse = 0
start_timer = time.time()

counter = 0

print('Initializing system')

SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

ser = serial.Serial("/dev/ttyS0", baudrate = 9600, timeout=1)

def calculate_elapse(channel):				
    global pulse, start_timer, elapse
    pulse+=1
    elapse = time.time() - start_timer
    start_timer = time.time()	

GPIO.add_event_detect(sensor, GPIO.FALLING, callback = calculate_elapse, bouncetime = 20)

myAPI = 'QZXBWYIDG9SFWTMT'
baseURL = 'https://api.thingspeak.com/update?api_key=%s' % myAPI

faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eyeCascade = cv2.CascadeClassifier('haarcascade_eye.xml')
 
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
#camera.start_preview()


ser.write('AT\r\n')
time.sleep(1)
ser.write('ATE0\r\n')
time.sleep(1)
ser.write('AT+CMGF=1\r\n')
time.sleep(1)
ser.write('AT+CNMI=2,1,0,0,0\r\n')
time.sleep(1)

ser.write('AT+CGPSPWR=1\r\n')       ######################
time.sleep(1)
ser.write('AT+CGPSRST=0\r\n')
time.sleep(1)


ser.write('AT+CMGS="8208060685"\r\n')
time.sleep(1)
ser.write('System ready\r\n')
time.sleep(1)
ser.write('\x1A')
time.sleep(1)

print('System ready\n\n')

def calculate_speed(r_cm):
    global pulse,elapse,rpm,dist_km,dist_meas,km_per_sec,km_per_hour
    if elapse !=0:		
        rpm = 1/elapse * 60
        circ_cm = (2*math.pi)*r_cm	
        dist_km = circ_cm/100000 	
        km_per_sec = dist_km / elapse		
        km_per_hour = km_per_sec * 3600		
        dist_meas = (dist_km*pulse)*1000	
        return km_per_hour

while True:
    accelerometer_x = mcp.read_adc(0)
    accelerometer_y = mcp.read_adc(1)
    accelerometer_z = mcp.read_adc(2)
    
    temperature_1 = mcp.read_adc(3)
    temperature = temperature_1 / 2
    print(temperature)
    alcohole = mcp.read_adc(4)

    rec = ''
    ser.write('AT+CGPSINF=0\r\n')
    time.sleep(1)
    rec = ser.read(ser.inWaiting())
    gps_data = rec.split(',')
    if len(gps_data) > 1:
        longitude = gps_data[1]
        latitude = gps_data[2]

    calculate_speed(20)

    stream = io.BytesIO()
    camera.capture(stream, format='jpeg')
    data = np.fromstring(stream.getvalue(), dtype=np.uint8)
    image = cv2.imdecode(data, 1)
    img = image[:, :, ::-1]
    img = np.array(img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.3,
        minNeighbors=5,      
        minSize=(30, 30)
    )

    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]

        print('face')
        counter = counter + 1
        
        eyes = eyeCascade.detectMultiScale(
            roi_gray,
            scaleFactor= 1.5,
            minNeighbors=10,
            minSize=(5, 5),
            )
        
        for (ex, ey, ew, eh) in eyes:
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)
            print('eye')
            counter = 0
               
        cv2.imshow('video', img)
    print(counter)

    if counter > 5:
        print('Person is sleeping')
        counter = 0
    
    print('x = {}\ty = {}\tz = {}\ttemp = {}\tal = {}\trpm = {}\tlong = {}\tlat = {}'.format(accelerometer_x, accelerometer_y, accelerometer_z, temperature, alcohole, rpm, longitude, latitude))
    conn = urllib2.urlopen(baseURL + '&field1=%s&field2=%s&field3=%s&field4=%s' % (temperature, alcohole, longitude, latitude))
    conn.close()
    if accelerometer_x < 280 or accelerometer_x > 400 or accelerometer_y < 280 or accelerometer_y > 400 or accelerometer_z < 280:    
        print('\n\nEmergency needed\nAccident happened at location : {}, {}\n\n'.format(longitude, latitude))
        ser.write('AT+CMGS="8208060685"\r\n')
        time.sleep(1)
        ser.write('Accident Happened at emergency needed\nLocation :\nLong: {}\nlat: {}\r\n'.format(longitude, latitude))
        time.sleep(1)
        ser.write('\x1A')
        time.sleep(1)
        
    switch_state = GPIO.input(switch)
    if switch_state == False and alcohole < 150:
        GPIO.output(motor, True)
        print('motor on')
    else:
        GPIO.output(motor, False)

    time.sleep(0.5)
