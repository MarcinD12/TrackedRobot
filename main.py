from machine import Pin, I2C, PWM
from pico_i2c_lcd import I2cLcd   #https://github.com/T-622/RPI-PICO-I2C-LCD
from hcsr04 import HCSR04    #https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib
from imu import MPU6050
import time
from radar import *
import network                  #https://docs.micropython.org/en/latest/esp8266/tutorial/network_basics.html
import socket
import urequests

#ctrl+shift+p select debugger

sta_if = network.WLAN(network.STA_IF)           #network connection
sta_if.active(True)
sta_if.connect('Starlink1','wifipass')
print(sta_if.isconnected())
print(sta_if.ifconfig())


led = Pin("LED",Pin.OUT)

i2c=I2C(id=1,scl=Pin(15),sda=Pin(14),freq=400000)       #display
lcd=I2cLcd(i2c, 0x3f, 2, 16)





#gyro
#mpu = MPU6050(i2c)
#print(round(mpu.accel.x,2))
#print(mpu.accel)
#xd = mpu.sensors
#print(xd[0].z)

#start = time.time()

# while (time.time()<start+10):
#     print(gyroRead.x)
#     print(gyroRead.y)
#     print(gyroRead.z)
#     print("next")
#     time.sleep_ms(1000)


leftMotorIn1=Pin(12,Pin.OUT)                         #motor driver
leftMotorIn2=Pin(11,Pin.OUT)
rightMotorIn1=Pin(10,Pin.OUT)
rightMotorIn2=Pin(9,Pin.OUT)
MotorPwm=PWM(Pin(8))
MotorPwm.freq(20)

distanceSensor= HCSR04(trigger_pin=7,echo_pin=6)      #radar distance sensor
radarMeasure= list()

def radar():
    servoPwm=PWM(Pin(13))                       #radar servo
    servoPwm.freq(50) 
   # i=2500  
    for i in range(2000,8500,500):              #mid = 5250
        radarDistance=distanceSensor.distance_cm()
        lcd.putstr(str(radarDistance))
        radarMeasure.append(radarDistance)
        servoPwm.duty_u16(i)
        time.sleep(0.5)
        lcd.clear()
    servoPwm.duty_u16(2000)
    time.sleep(1)
    servoPwm.deinit()
    lcd.clear()
    direction = radarMeasure.index(max(radarMeasure)) 
    print(direction)
    return(direction)
    



def servoTest():                #between 2500-8500
    servoPwm.duty_u16(1500)
    time.sleep(1)
    servoPwm.duty_u16(8500)
    time.sleep(1)
    servoPwm.deinit()




"""     Motor steering:
        IN1    IN2    PWM   
        1       0      x   -Max PWM CW
        0       1      x   -Max PWM-CCW
        0       0      1   -Fast stop
        1       1      1   -Soft stop   
"""
def moveBackward(timeOfMove):
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(1)
    leftMotorIn2.value(0)
    rightMotorIn1.value(1)
    rightMotorIn2.value(0)
    time.sleep(timeOfMove)
    moveStop()

def moveForward(timeOfMove):
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(0)
    leftMotorIn2.value(1)
    rightMotorIn1.value(0)
    rightMotorIn2.value(1)
    time.sleep(timeOfMove)
    moveStop()
    


def turnLeft(timeOfMove):
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(1)
    leftMotorIn2.value(0)
    rightMotorIn1(0)
    rightMotorIn2(1)
    time.sleep(timeOfMove)
    moveStop()

def turnRight(timeOfMove):
    MotorPwm.duty_u16(power)
    rightMotorIn1.value(1)
    rightMotorIn2.value(0)
    leftMotorIn1(0)
    leftMotorIn2(1)
    time.sleep(timeOfMove)
    moveStop()

def moveStop():
    leftMotorIn1.value(0)
    leftMotorIn2.value(0)
    rightMotorIn1.value(0)
    rightMotorIn2.value(0)


def drivesTest(timeOfMove):
    MotorPwm.duty_u16(power)
    moveForward(timeOfMove)
    moveBackward(timeOfMove)  
    turnLeft(timeOfMove)
    turnRight(timeOfMove)
    

# power = 15000
# moveForward(1)
# moveBackward(1)
# turnLeft(1)
# turnRight(1)
# moveStop()

power=15000

#program loop
for i in range(1000):
    
    lcd.move_to(0,0)
    request = urequests.get('apiip/robot/info')   
    command = request.text
    print(command+"        ")
    lcd.putstr(str(command))
    if command=="Forward":
        moveForward(0.5)
    elif command=="Backward":
        moveBackward(0.5)
    elif command=="Left":
        turnLeft(0.5)
    elif command=="Right":
        turnRight(0.5)
    else:
        moveStop()




