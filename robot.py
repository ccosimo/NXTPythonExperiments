#!/usr/bin/env python

import nxt
import thread 
import time
import datetime 
from nxt.sensor import *


myBrickHost = '00:16:53:02:8D:24'
ONEDGELIMIT = 16
STEPSPEED = 60
STEPROTATION = 90

def idle():
    time.sleep(0.2) # value is in secs
    # idle()

def turnmotor(motor, lck, power, degrees):

    while lck.locked():
        idle()
    lck.acquire()

    motor.turn(power, degrees)
    
    lck.release()
    # turnmotor()

def forward():
    while left_lock.locked() or right_lock.locked():
        idle()

    thread.start_new_thread(turnmotor, (left_motor, left_lock, STEPSPEED, STEPROTATION))
    thread.start_new_thread(turnmotor, (right_motor, right_lock, STEPSPEED, STEPROTATION))

    while left_lock.locked() or right_lock.locked():
        idle()
    
    # forward()

def backward():
    while left_lock.locked() or right_lock.locked():
        idle()

    thread.start_new_thread(turnmotor, (left_motor, left_lock, -1 * STEPSPEED, STEPROTATION))
    thread.start_new_thread(turnmotor, (right_motor, right_lock, -1 * STEPSPEED, STEPROTATION))

    while left_lock.locked() or right_lock.locked():
        idle()
    
    # backward()

def right():
    while left_lock.locked() or right_lock.locked():
        idle()

    thread.start_new_thread(turnmotor, (left_motor, left_lock, STEPSPEED, STEPROTATION))
    thread.start_new_thread(turnmotor, (right_motor, right_lock, -1 * STEPSPEED, STEPROTATION))

    while left_lock.locked() or right_lock.locked():
        idle()

    # right()

def left():
    while left_lock.locked() or right_lock.locked():
        idle()

    thread.start_new_thread(turnmotor, (right_motor, right_lock, STEPSPEED, STEPROTATION))
    thread.start_new_thread(turnmotor, (left_motor, left_lock, -1 * STEPSPEED, STEPROTATION))

    while left_lock.locked() or right_lock.locked():
        idle()

    # left()

def log(msg):
    if msg == '':
        logFile.write('\r\n')
    else:
        logFile.write(str(datetime.datetime.now()) + " > " + msg + '\r\n')
    
    # log()

# get limit proximity itaration on a separated thread
# each check is separated by an idle() cycle
# parameters:
# sensor, ultrasonic sensor object
# status, sensor last value read; is a vector, position 0 is the left sensor, position 1 is the right
#         moreover, in this way the parameter is used as object and passed by reference
def checkEdge(leftSensor, rightSensor, status):
    while True:
        try:
            status[0] = leftSensor.get_sample()
        except:
            log('exception reading left edge sensor, keep previous value')
            
        try:
            status[1] = rightSensor.get_sample()
        except:
            log('exception reading left edge sensor, keep previous value')

        idle()

    # checkEdge()

logFile = open('robot.log', 'a+')

log ('')

log('starting...')

log('NXT brick acquiring')

#nxt.locator.make_config()
b = nxt.locator.find_one_brick(host = myBrickHost, debug = True)
#b = nxt.locator.find_one_brick()
left_motor = nxt.Motor(b, nxt.PORT_A)
right_motor = nxt.Motor(b, nxt.PORT_B)

left_lock = thread.allocate_lock();
right_lock = thread.allocate_lock();

log('NXT brick acquired')

log('sensors acquiring')

leftEdgeSensor = nxt.Ultrasonic(b, nxt.PORT_4, False)
rightEdgeSensor = nxt.Ultrasonic(b, nxt.PORT_3, False)
endSensor = nxt.Touch(b, nxt.PORT_1)

log('sensors acquired')

log('start edge sensor thread')
edgeStatus = [0, 0]
thread.start_new_thread(checkEdge, (leftEdgeSensor, rightEdgeSensor, edgeStatus))

log('start loop')
end = False
while not end:
    
    if not end: # no stop button yet
        end = endSensor.get_sample()

        if not end: # no stop button again
            if (edgeStatus[0] > ONEDGELIMIT) or (edgeStatus[1] > ONEDGELIMIT): # the limit was passed
                backward()
                idle()
                backward()
                idle()
                right()
                idle()
                
            else: # not yet reached the limit, go ahead
                forward()
            
        else: # stop button, little pause to permit button release
            time.sleep(2)
            log('start sleep')

    else: # stop still in progress
       
        if endSensor.get_sample(): # button pressed, start again
            end = False
            time.sleep(2)
            log('end sleep')

logFile.close()

