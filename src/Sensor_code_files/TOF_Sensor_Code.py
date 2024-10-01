import busio
import board
import adafruit_vl53l0x as ToF
import time
import RPi.GPIO as GPIO

shutdown_1 = 4 # XSHUT pin 1 on GPIO 4
shutdown_2 = 17 # XSHUT pin 2 on GPIO 17
shutdown_3 = 27 # XHSUT pin 3 on GPIO 27

#initialise i2c
i2c = busio.I2C(board.SCL, board.SDA)

### Set up GPIO pins ###
# Set RPi.GPIO to read GPIO No. instead of Pin No.
GPIO.setup(GPIO.BCM)
GPIO.setup(shutdown_1,GPIO.OUT) # set GPIO 4 OUT
GPIO.setup(shutdown_2,GPIO.OUT) # set GPIO 17 OUT
GPIO.setup(shutdown_3,GPIO.OUT) # set GPIO 27 OUT

# Initialise sensor reset (sensor XSHUT active when LOW)
GPIO.output(shutdown_1,GPIO.LOW)
GPIO.output(shutdown_2,GPIO.LOW)
GPIO.output(shutdown_3,GPIO.LOW)
time.sleep(0.1)
GPIO.output(shutdown_1,GPIO.HIGH)
GPIO.output(shutdown_2,GPIO.HIGH)
GPIO.output(shutdown_3,GPIO.HIGH)
print("reset done")

### set up sensor addresses ###
# sensor 1 address
GPIO.output(shutdown_1,GPIO.HIGH)
GPIO.output(shutdown_2,GPIO.LOW)
GPIO.output(shutdown_3,GPIO.LOW)
sens1 = ToF.VL53L0X(i2c) # read i2c on sensor 1
sens1.set_address(0x30) # set address to 0x30
print("sensor 1 with address 0x30")
time.sleep(0.1)

# sensor 2 address
GPIO.output(shutdown_2,GPIO.HIGH) # bring sensor 2 HIGH
sens2 = ToF.VL53L0X(i2c)
sens2.set_address(0x31)
print("sensor 2 with address 0x31")
time.sleep(0.1)

# sensor 3 address
GPIO.output(shutdown_3,GPIO.HIGH)
sens3 = ToF.VL53L0X(i2c)
sens3.set_address(0x32)
print("sensor 3 with address 0x30")

#print statement to prove compeletion
print("Reset Complete")

def read():
    while True:
        sensor_1 = sens1.range # varaible to store sensor 1 range
        if sensor_1 < 100:
            print("stop")
        else:
            print("Sensor 1 Range : {}mm".format(sensor_1))

        sensor_2 = sens2.range # varaible to store sensor 1 range
        if sensor_2 < 100:
            print("stop")
        else:
            print("Sensor 1 Range : {}mm".format(sensor_2))

        sensor_3 = sens3.range # varaible to store sensor 1 range
        if sensor_3 < 100:
            print("stop")
        else:
            print("Sensor 1 Range : {}mm".format(sensor_3))
        
        #small delay for processing
        time.sleep(1)


''' Run Code '''
read()