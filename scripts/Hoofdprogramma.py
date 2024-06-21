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
    rospy.init_node('hoofdprogramma/hmi_signal', Bool, queue_size=10))
    if HMI_Subscribers.data == TRUE:
        print("Received start signal from HMI.")
        start_Camera launchfile
        start_manupulator launchfile
    else:
        print("Received stop signal from HMI.")
        
def choice_callback(self, msg):
        # This function will be called every time a message is received on 'hmi_choice' topic
        choice = msg.data
        
        # Define actions based on the received choice using if statements
        if choice == 1: 
            rospy.loginfo("Sorteer: Dop_10")
            # Add your specific action for Option 1 here
        
        elif choice == 2:
            rospy.loginfo("Sorteer: KleineSchroevendraaier")
            # Add your specific action for Option 2 here
        
        elif choice == 3:
            rospy.loginfo("Sorteer: Plattekop")
            # Add your specific action for Option 3 here
        
        elif choice == 4:
            rospy.loginfo("Sorteer: Spanningstester")
            # Add your specific action for Option 4 here
        
        elif choice == 5:
            rospy.loginfo("Alles sorteren")
            # Add your specific action for Option 5 here
        
        else:
            rospy.logwarn("Received unknown option: %d", choice)
            # Handle unexpected options or add additional logic as needed        

#ontvangen en verwerken start signaal

#keuzes verwerken


#vision aansturing en verwerking
vision: 

+starten camera

// if object = found send info to manipulation

argumenten
float x, float y, float z, float angle
int class index 



#manipulatie aansturing en verwerking 
manupilator
x,y,z yawn 
+startsignaal



if __name__ == '__main__':
  subscriber_node()
  main()
