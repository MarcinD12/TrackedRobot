from machine import Pin, I2C, PWM
import machine
from pico_i2c_lcd import I2cLcd   #https://github.com/T-622/RPI-PICO-I2C-LCD
from hcsr04 import HCSR04    #https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib
from imu import MPU6050
import time
try:
    from pico_i2c_lcd import I2cLcd
except:
    from pico_i2c_lcd import I2cLcd  
import network                  #https://docs.micropython.org/en/latest/esp8266/tutorial/network_basics.html
import urequests
import ntptime
import urequest
import json
import secrets
#import urequests
#ctrl+shift+p select debugger

  
sta_if = network.WLAN(network.STA_IF)           #network connection
sta_if.active(True)
sta_if.connect(secrets.SSID,secrets.PASSWORD)
print(sta_if.isconnected())
print(sta_if.ifconfig())


led = Pin("LED",Pin.OUT)
i2c=I2C(id=1,scl=Pin(15),sda=Pin(14),freq=400000)       #display
lcd=I2cLcd(i2c, 0x3f, 2, 16)

resetBtn = Pin(16,Pin.IN,Pin.PULL_DOWN)

leftMotorIn1=Pin(12,Pin.OUT)                         #motor driver
leftMotorIn2=Pin(11,Pin.OUT)
rightMotorIn1=Pin(10,Pin.OUT)
rightMotorIn2=Pin(9,Pin.OUT)
MotorPwm=PWM(Pin(8))
MotorPwm.freq(20)

distanceSensor= HCSR04(trigger_pin=7,echo_pin=6)      #radar distance sensor

def WakeUp():
    for i in range(10):
        lcd.putstr('READY')
        led.toggle()
        time.sleep(1)
        lcd.clear()

#gyro
mpu = MPU6050(i2c)
def TurnDegree(degree):
    givenDegree=int(degree)
    print('inside')
    if givenDegree>0:
        turnRight()
    else:
        turnLeft()
    print('direction set')
    start = time.time()
    previous_time=time.ticks_ms()
    current_angle=0.0
    time_class=machine.RTC()
    print('before loop')
    print()
    while (abs(current_angle)<abs(givenDegree)):
        print('in loop')
        print('next----------')
        sample = mpu.gyro.z
        print('sample'+str(sample))
        time_sample=time.ticks_ms()
        #time_sample=time_class.datetime()[7]
        print('time_sample'+str(time_sample))
        delta_time = time.ticks_diff(time_sample,previous_time)/1000
        print('delta_time'+str(delta_time))
        delta_angle=(sample)*(delta_time)
        print(str(delta_angle))
        current_angle=current_angle+delta_angle
        print('angle'+str(current_angle))
        previous_time=time_sample
        print('pretime'+str(previous_time))
        print('angle set')
        
    moveStop()    


radarMeasure= list()

def Radar():
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
    



def ServoTest():                #between 2500-8500
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
def MoveBackward(timeOfMove):
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(1)
    leftMotorIn2.value(0)
    rightMotorIn1.value(1)
    rightMotorIn2.value(0)
    time.sleep(timeOfMove)
    moveStop()

def MoveForward(timeOfMove):
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(0)
    leftMotorIn2.value(1)
    rightMotorIn1.value(0)
    rightMotorIn2.value(1)
    time.sleep(timeOfMove)
    moveStop()
    


def TurnLeft():
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(1)
    leftMotorIn2.value(0)
    rightMotorIn1(0)
    rightMotorIn2(1)
    
def TurnRight():
    MotorPwm.duty_u16(power)
    rightMotorIn1.value(1)
    rightMotorIn2.value(0)
    leftMotorIn1(0)
    leftMotorIn2(1)   
    

def MoveStop():
    leftMotorIn1.value(0)
    leftMotorIn2.value(0)
    rightMotorIn1.value(0)
    rightMotorIn2.value(0)


def DrivesTest(timeOfMove):
    MotorPwm.duty_u16(power)
    moveForward(timeOfMove)
    moveBackward(timeOfMove)  
    turnLeft(timeOfMove)
    turnRight(timeOfMove)
    
def Interruption(resetBtn):
    MoveStop()
    lcd.putstr('RESETING BOARD')
    print('RESET')
    machine.reset()

resetBtn.irq(trigger=Pin.IRQ_RISING,handler=Interruption)

# power = 15000
# moveForward(1)
# moveBackward(1)
# turnLeft(1)
# turnRight(1)
# moveStop()
power=15000
#TurnDegree(90)

MoveStop()
#TurnDegree(90)
#program loop
def ServerLoop():
    for i in range(1000):
        
        lcd.move_to(0,0)
        print("time before: "+str(time.localtime()))

        response =urequests.get(secrets.APIADRESS)  
        dataPackage=json.loads(response.text)
        #command=response.text
        print(dataPackage)
        command=dataPackage['command']
        lcd.clear()
        lcd.putstr(str(command))

        curtime=time.localtime()
        print("timeafter: "+str(curtime))
        print(command+"        ")
        lcd.putstr(str(command)+"       ")

        if command=="forward":
            moveForward(3)
        elif command=="backward":
            moveBackward(3)
        elif command=="left":
            TurnDegree(dataPackage['angle'])
        elif command=="right":
            TurnDegree(dataPackage['angle'])
        elif command=="halfleft":
            TurnDegree(dataPackage['angle'])
        elif command=="halfright":
            TurnDegree(dataPackage['angle'])
        else:
            moveStop()
#ServerLoop()

WakeUp()

for i in range(10):
    lcd.putstr('waiting')
    time.sleep(0.5)
    lcd.clear()

