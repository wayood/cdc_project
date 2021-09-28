#!/usr/bin/env python                                                           
import rospy
from ros_beginner.srv import DateTrigger, DateTriggerResponse
from datetime import datetime

def callback_srv(data):                                          #changed
    l = []
    d = DateTriggerResponse()                                                    
    d.date = ''                                                  #9~12 changed
    d.time = ''
    try:                                                         #add
        now = datetime.now()
        l = str(now)

        for i in range(0,10):
            d.date += l[i]
        for i in range(11,25):
            d.time += l[i]
        d.date = int(d.date.replace('-', ''))
        d.time = float(d.time.replace(':',''))
        d.success = True                                        #22~25 changed
    except:
        d.success = False
    return d

if __name__ == '__main__':
    rospy.init_node('date_server')                              #changed
    srv = rospy.Service('date_call', DateTrigger, callback_srv) #changed
    rospy.spin()