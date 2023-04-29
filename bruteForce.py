from maestro import Controller
import time 

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3

tango = Controller()

motors = 6700
turn = 5300
tango.setTarget(MOTORS,motors)
tango.setTarget(TURN, turn)



motors = 6000
turn = 6000
tango.setTarget(MOTORS,motors)
tango.setTarget(TURN, turn)