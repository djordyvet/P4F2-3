"
Gemaakt door:       Pieter Roozendaal
Datum:              21-6-2024
Prgmma:             Hoofdprogramma.py
"
#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

#subscriber nodes
def HMI_Subscribers():
    rospy.init_node('hmi_signal_subscriber', anonymous=True)
    rospy.Subscriber('hmi_signal', Bool, signal_callback)
    rospy.spin()
#subscribers voor Vision


#subscribers voor Manipulatie


////
#publisher nodes
#publisher voor HMI

#publisher voor Vision

#publisher voor Manipulatie

////
#HMI aansturing en verwerking
///
Start signaal verwerken
def signal_callback(msg):
    if HMI_Subscribers.data == TRUE:
        /// send signal to start camera
          
      print("Received start signal from HMI.")
        # Add your start process logic here
        # For example, start a process or initiate some action
    else:
        print("Received stop signal from HMI.")
        # Add your stop process logic here
        # For example, stop a process or perform shutdown actions


#ontvangen en verwerken start signaal

#keuzes verwerken


#vision aansturing en verwerking
vision: 

// if object = found send info to manipulation

argumenten
float x, float y, float z, float angle
int class index 



#manipulatie aansturing en verwerking 
manupilator
x,y,z yawn 
startsignaal



if __name__ == '__main__':
  subscriber_node()
  main()
