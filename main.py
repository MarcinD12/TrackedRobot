from machine import Pin, I2C, PWM
import machine
from pico_i2c_lcd import I2cLcd   #https://github.com/T-622/RPI-PICO-I2C-LCD
from hcsr04 import HCSR04    #https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib
from imu import MPU6050
import time
import network                  #https://docs.micropython.org/en/latest/esp8266/tutorial/network_basics.html
import ntptime
import urequest
import json
import secrets
import socket

#ctrl+shift+p select debugger

rp2.country('PL')

sta_if = network.WLAN(network.STA_IF)           #network connection
sta_if.config(pm = 0xa11140)
#print(str(sta_if.config(pm = 0xa11140)))
sta_if.active(True)

sta_if.connect(secrets.SSID,secrets.PASSWORD)
print(sta_if.isconnected())
print(sta_if.ifconfig())


led = Pin("LED",Pin.OUT)
i2c=I2C(id=1,scl=Pin(15),sda=Pin(14),freq=400000)       #display
lcd=I2cLcd(i2c, 0x3f, 2, 16)

resetBtn = Pin(2,Pin.IN,Pin.PULL_UP)

leftMotorIn1=Pin(12,Pin.OUT)                         #motor driver
leftMotorIn2=Pin(11,Pin.OUT)
rightMotorIn1=Pin(10,Pin.OUT)
rightMotorIn2=Pin(9,Pin.OUT)
MotorPwm=PWM(Pin(8))
MotorPwm.freq(20)
power=20000

distanceSensor= HCSR04(trigger_pin=7,echo_pin=6)      #radar distance sensor

def WakeUp():
    for i in range(3):
        lcd.putstr('READY')
        led.toggle()
        time.sleep(1)
        lcd.clear()

#gyro
mpu = MPU6050(i2c)
def TurnDegree(degree):
    givenDegree=int(degree)
    if givenDegree>0:
        TurnRight()
    else:
        TurnLeft()
    print('direction set')
    start = time.time()
    previous_time=time.ticks_ms()
    current_angle=0.0
    time_class=machine.RTC()
    while (abs(current_angle)<abs(givenDegree)):
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
    MoveStop()    




def RadarScan():
    radarData= list()
    servoPwm=PWM(Pin(13))                       #radar servo
    servoPwm.freq(50) 
   # i=2500  
    for i in range(2000,8500,500):              #mid = 5250
        radarDistance=distanceSensor.distance_cm()
        lcd.putstr(str(radarDistance))
        radarData.append(radarDistance)
        servoPwm.duty_u16(i)
        time.sleep(0.5)
        lcd.clear()
    servoPwm.duty_u16(2000)#not leaving servo in edge position
    time.sleep(1)
    servoPwm.deinit()
    lcd.clear()
    #irection = radarData.index(max(radarData)) 
    print(radarData)

    return(radarData)
    
def RadarGetDirection(data):
    return(radarData.index(max(radarData)))


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
def MoveBackward():
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(1)
    leftMotorIn2.value(0)
    rightMotorIn1.value(1)
    rightMotorIn2.value(0)

def MoveForward():
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(0)
    leftMotorIn2.value(1)
    rightMotorIn1.value(0)
    rightMotorIn2.value(1)

    


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

resetBtn.irq(trigger=Pin.IRQ_FALLING,handler=Interruption)




def ApiPostRadarData(scanValues):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #scanValuestest=[2,1,3,7]
    #print(json.dumps(scanValuestest).encode('utf8'))
    msg=json.dumps(scanValues).encode('utf8')
    dataLen=len(msg) 
    print(msg)
    print(dataLen)
    requestHeaders= f"POST /robot/radarData HTTP/1.1\r\nHost: {secrets.APIADRESS}\r\nContent-Length: {dataLen}\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"
    requestBody=msg
    s.connect((f'{secrets.APIADRESS}',30583))
    #s.send((f"POST /robot/radarData HTTP/1.1\r\nHost: {secrets.APIADRESS}\r\nContent-Length:{str(dataLen)}\r\nContent-Type:application/json\r\nConnection:close\r\n\r\n {msg}\r\n"))
    s.send(requestHeaders.encode('utf8')+requestBody+b'\r\n')
    response = s.recv(4096)
    print(response)
    s.close()

def RadarTest():
    for i in range(2):
        ApiPostRadarData(RadarScan())


def ApiGet():
    for i in range(200):
        #print("time before: "+str(time.localtime()))
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((f'{secrets.APIADRESS}',30583))
        s.send((f"GET /robot/getcommands HTTP/1.1\r\n Host: {secrets.APIADRESS} \r\n\r\n"))
        response = s.recv(4096)
        s.close()
        body=(response.splitlines()[-1])
        body.decode('utf8')
        res=json.loads(body)
        print(res["command"])
        command=res["command"]
        lcd.move_to(0,0)
        lcd.putstr(str(command))

        if command=="forward":
            MoveForward()
        elif command=="stop":
            MoveStop()
        elif command=="backward":
            MoveBackward()
        elif command=="left":
            TurnLeft()
        elif command=="right":
            TurnRight()
        elif command=="halfleft":
            TurnDegree(res['angle'])
        elif command=="halfright":
            TurnDegree(res['angle'])
        else:
            MoveStop()
WakeUp()
MoveStop()
ApiGet()
MoveStop()
#RadarTest()


# for i in range(10):
#     lcd.putstr('waiting')
#     time.sleep(0.5)
#     lcd.clear()

