import tkinter as tk
from maestro import Controller                                                     

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3


##def sendCommand(x):
##    if(x == '8'):
##        tango.setTarget(MOTOR, 6800)
class KeyControl():
    def __init__(self,win):
        self.root = win
        self.tango = Controller()
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000

    #114 == right
    #113 == left
    #111 == forward
    #116 == backward


    def iDoNothing():
        print("")
 
    def arrow(self, key):
        print(key.keycode)
        if key.keycode == 116:
            self.motors += 200
            self.turn += 200
            if(self.motors > 7900):
                self.motors = 7900
            if(self.turn > 7900):
                self.turn = 7900
            print(self.motors)
            self.tango.setTarget(MOTORS, self.motors)
            self.tango.setTarget(TURN, self.turn)
        elif key.keycode == 111:
            self.motors -= 200
            if(self.motors < 1510):
                self.motors = 1510
            print(self.motors)
            self.tango.setTarget(MOTORS, self.motors)
        elif key.keycode == 114:
            self.turn += 200
            if(self.turn > 7400):
                self.turn = 7400
            print(self.turn)
            self.tango.setTarget(TURN, self.turn)
        elif key.keycode == 113:
            self.turn -= 200
            if(self.turn <2110):
                self.turn = 2110
            print(self.turn)
            self.tango.setTarget(TURN, self.turn)
        
        elif key.keycode == 65:
            self.motors = 6000
            self.turn = 6000
            self.tango.setTarget(MOTORS, self.motors)
            self.tango.setTarget(TURN, self.turn)

win = tk.Tk()
keys = KeyControl(win)

win.bind('<Up>', keys.arrow)
win.bind('<Left>', keys.arrow)
win.bind('<Down>', keys.arrow)
win.bind('<Right>', keys.arrow)
win.bind('<space>', keys.arrow)
win.mainloop()
keys = KeyControl(win)     
