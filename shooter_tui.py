import npyscreen
import signal
import sys
import time
from os import system

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from m2_pr2020.srv import SetShoot
# This application class serves as a wrapper for the initialization of curses
# and also manages the actual forms of the application

START = -3000
RELEASE_PT = -7000
STOP = -15000
ready = False
RANGE = 1000

RPM = 4000
ACC = 9000
DEC = 11000
IO = 0

def shoot_cb(data):
    global ready
    pos = abs(data.x)
    if ready:
        #print(pos)
        if pos >= (abs(RELEASE_PT) - RANGE ) and pos <= (abs(RELEASE_PT) + RANGE ):
            clip_io_pub.publish(True)
            print("Release")

class MyApp(npyscreen.NPSAppManaged):
    def onStart(self):
        self.registerForm("MAIN", MainForm())
        self.registerForm("Page_2", Page_2())
# This form class defines the display that will be presented to the user.

class MainForm(npyscreen.ActionFormV2):
    OK_BUTTON_TEXT = "Go"
    CANCEL_BUTTON_TEXT = "Quit"
 

    
    def create(self):
        self.add(npyscreen.TitleFixedText, name = "Select IO port", value= "Select IO in use" )
        self.io = self.add(npyscreen.TitleSelectOne,name="IO in use",values=["io_1","io_2","io_3","io_4","io_5","io_6","io_7","io_8"],scroll_exit=True)


    def go(self,items):
        pass


    def on_ok(self):
        msg=""
        selected_io=(self.io.get_selected_objects())
        if len(selected_io)>0:
            IO = selected_io[0]
            clip_io_pub = rospy.Publisher('/io_board1/'+str(IO)+'/set_state', Bool, queue_size=1)
            msg+="IO in use : " + str(IO) + "\n"
            confirmed = npyscreen.notify_ok_cancel(msg, editw=1)
            self.parentApp.setNextForm("Page_2")
        #self.go(item)
    
    def on_cancel(self):
        confirmed = npyscreen.notify_ok_cancel("Quit?", editw=1)
        if confirmed:
            exit(0)

class Page_2(npyscreen.ActionFormV2):
    OK_BUTTON_TEXT = "Go"
    CANCEL_BUTTON_TEXT = "Quit"



    def create(self):
        shooter_shoot_srv = rospy.ServiceProxy('shooter/trigger_shoot_pos', SetShoot)
        rospy.Subscriber("motor_shooter/p_feedback", Vector3, shoot_cb, queue_size=1)
        self.rpm=self.add(npyscreen.TitleSlider, name = "rpm", value = 12, step=1, rely = 5, out_of = 24, max_width=120)
        self.acc=self.add(npyscreen.TitleSlider, name = "acceleration",  value = 10, step=1, rely = 10, out_of = 10, max_width=120)
        self.dec=self.add(npyscreen.TitleSlider, name = "deceleration", value = 10, step=1, rely = 15, out_of = 10, max_width=120)

        



    def go(self,items):
        pass


    def on_ok(self):
        selected_rpm=(self.rpm.get_value())
        selected_acc=(self.acc.get_value())
        selected_dec=(self.dec.get_value())
        msg="rpm : " + str(selected_rpm) + "\nacceleration : " + str(selected_acc) + "\ndeceleration : " + str(selected_dec) + "\n Ready to shoot?"
        confirmed = npyscreen.notify_ok_cancel(msg, editw=1)

        if confirmed:
            shooter_shoot_srv(START, 1000, 1000, 1000)
            rospy.sleep(3)
            ready = True
            shooter_shoot_srv(STOP, RPM*(selected_rpm/24) , ACC*(selected_acc/10), DEC*(selected_dec/10) )
            rospy.sleep(2)
            shooter_shoot_srv(START, 1000, 1000, 1000)
        #self.go(item)
    
    def on_cancel(self):
        confirmed = npyscreen.notify_ok_cancel("Quit?", editw=1)
        if confirmed:
            exit(0)



def signal_handler(sig, frame):
    pass


#def TurnOn(num):

#def TurnOff(num):


if __name__ == '__main__':
    rospy.init_node('shooter_tui')
    TA = MyApp()
    TA.run()
