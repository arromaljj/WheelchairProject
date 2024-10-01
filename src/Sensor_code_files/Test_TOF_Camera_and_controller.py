from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO
import busio
import board
import adafruit_vl53l0x as ToF
import time
import RPi.GPIO as GPIO
import threading
import cv2

''' Global Variables '''
# TPI Pins
forward = 21 #forward quadrant on GPIO 21
backward = 20 #backward quadrant on GPIO 20
right = 26 #right quadrant on GPIO 26
left = 16 #left quadrant on GPIO 16
emerge_stop = 19 # emergency stop on GPIO 19
# Shutdown Pins
shutdown_1 = 4 # shutdown pin 1 on GPIO 4
shutdown_2 = 17 #shutdown pin 2 on GPIO 17
shutdown_3 = 27 #shutdown pin 3 on GPIO 27
''' add more pins as required '''
#initialise i2c
i2c = busio.I2C(board.SCL,board.SDA)

''' Set Up '''
# RPi.GPIO to use GPIO No. instead of Pin No.
GPIO.setup(GPIO.BCM)
### GPIO Output ###
GPIO.setup(forward,GPIO.OUT)
GPIO.setup(right,GPIO.OUT)
GPIO.setup(backward,GPIO.OUT)
GPIO.setup(left,GPIO.OUT)
GPIO.setup(emerge_stop,GPIO.OUT)
### GPIO Initial Values ###
GPIO.output(forward,1)
GPIO.output(right,1)
GPIO.output(backward,1)
GPIO.output(left,1)
GPIO.output(emerge_stop,0)

### Set reset pins on ToF sensors ###
GPIO.output(shutdown_1, GPIO.LOW)
GPIO.output(shutdown_2, GPIO.LOW)
GPIO.output(shutdown_3, GPIO.LOW)
time.sleep(0.1)
GPIO.output(shutdown_1, GPIO.HIGH)
GPIO.output(shutdown_2, GPIO.HIGH)
GPIO.output(shutdown_3, GPIO.HIGH)
print("reset done")

### Sensor Addresses ###
# Sensor 1 Address
GPIO.output(shutdown_1, GPIO.HIGH) # keep sensor 1 high
GPIO.output(shutdown_1, GPIO.LOW)
GPIO.output(shutdown_1, GPIO.LOW)
sens1 = ToF.VL53L0X(i2c) # read i2c of sensor 1
sens1.set_address(0x30) # set new address for sensor 1
print("sensor 1 with address 0x30")
time.sleep(0.1)

# Sensor 2 Address
GPIO.output(shutdown_1, GPIO.HIGH) # make sensor 2 high
sens2 = ToF.VL53L0X(i2c)
sens2.set_address(0x31) # set new address of sensor 2
print("sensor 2 with address 0x31")
time.sleep(0.1)

# Sensor 3 Address
GPIO.output(shutdown_1, GPIO.HIGH) # make sensor 3 HIGH
sens3 = ToF.VL53L0X(i2c)
sens3.set_address(0x32)
print("sensor 3 with address 0x32")

# show addresses are complete
print("Reset Complete")

### Control Button Read ###
class MyController(Controller):

    def __init__(self,**kwargs):
        Controller.__init__(self,**kwargs)
    
    def on_up_arrow_press(self):
        print('Move Forward (Green LED)')
        GPIO.output(forward,0)
    
    def on_down_arrow_press(self):
        print('Move right (Red LED)')
        GPIO.output(right,0)
    
    def on_right_arrow_press(self):
        print('Rotate backward (Yellow LED)')
        GPIO.output(backward,0)
    
    def on_left_arrow_press(self):
        print('Rotate Anti-clockwise')
        GPIO.output(left,0)
    
    def on_up_down_arrow_release(self):
        print('Stop')
        GPIO.output(forward,1)
        GPIO.output(backward,1)
    
    def on_right_left_arrow_press(self):
        print('Stop')
        GPIO.output(right,1)
        GPIO.output(left,1)
    
    def on_circle_press(self):
        print('Module OFF')
        GPIO.output(emerge_stop,1)
    
    def on_square_press(self):
        print('Module ON')
        GPIO.output(emerge_stop,0)

controller = MyController(interface="/dev/input/js0",connecting_using_ds4drv=False)

### Camera Set Up ###
#print(cv2.__version__)

cv2.namedWindow("preview")
vc = cv2.VideoCapture(-1)

if vc.isOpened():
    rval, frame = vc.read()
else:
    rval = False

cv2.destroyWindow("preview")
vc.release

def read():
    ### sensor 1 statement and output ###
    sensor_1 = sens1.range # create variable to store sensor 1 distance
    if (sensor_1 < 100):
        print("stop")
    else:
        print("Sensor 1 Range: {}mm".format(sensor_1))

    ### sensor 1 statement and output ###
    sensor_2 = sens2.range # create variable to store sensor 1 distance
    if (sensor_2 < 100):
        print("stop")
    else:
        print("Sensor 1 Range: {}mm".format(sensor_2))

    ### sensor 1 statement and output ###
    sensor_3 = sens3.range # create variable to store sensor 1 distance
    if (sensor_3 < 100):
        print("stop")
    else:
        print("Sensor 1 Range: {}mm".format(sensor_3))
    
    #small delay
    time.sleep(1)


''' Main Code '''
while True and rval:
    #listen to controller
    controller.listen()

    #camera run
    cv2.imShow("preview",frame)
    rval, frame = vc.read()
    key = cv2.waitKey(20)
    if key == 27: #exit on esc
        break

    #ToF sensors
    read()
