#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time

class Demo1(object):
	def __init__(self):
		rospy.on_shutdown(self.cleanup)		
		rospy.init_node('demo1_node', anonymous=True)

		self.motor_pub = rospy.Publisher('cmd_vel', Twist)

		rospy.Subscriber("/emergency_stop", Bool, self.callbackEmergency)
		self.emergency = False

	def cleanup(self):
		# stop the robot
		twist = Twist()
		self.motor_pub.publish(twist)

	def callbackEmergency(self, emergency):
		if(emergency.data == True and self.emergency == False):
			self.emergency = True				
		
		elif(emergency.data == False and self.emergency == True):
			self.emergency= False

	def Forward(self, speed):
		# create a twist message, fill in the details
		twist = Twist()
		twist.linear.x = speed;                   # our forward speed
		twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
		twist.angular.x = 0; twist.angular.y = 0;   #          or these!
		twist.angular.z = 0;                        # no rotation
		self.motor_pub.publish(twist)	

	def Backward(self, speed):
		# create a twist message, fill in the details
		twist = Twist()
		twist.linear.x = speed * -1;                # our backward speed
		twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
		twist.angular.x = 0; twist.angular.y = 0;   #          or these!
		twist.angular.z = 0;                        # no rotation
		self.motor_pub.publish(twist)	

	def TurnLeft(self,speed):
		# create a twist message, fill in the details
		twist = Twist()
		twist.linear.x = 0;                         # no forward or backward speed
		twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
		twist.angular.x = 0; twist.angular.y = 0;   #          or these!
		twist.angular.z = speed * -1;               # rotation
		self.motor_pub.publish(twist)	

	def TurnRight(self):
		pass

	def Stop(self):
		# create a twist message, fill in the details
		twist = Twist()
		twist.linear.x = 0;                   	    # no speed
		twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
		twist.angular.x = 0; twist.angular.y = 0;   #          or these!
		twist.angular.z = 0;                        # no rotation
		self.motor_pub.publish(twist)	

	def Run(self, speed_x):		
		while True:	
			print "forward"	
			
			while (self.emergency == False):
				self.Forward(speed_x)
				pass

			print "Found an object, stopping robot"

			# emergency issued, stop robot
			self.Stop()		

			print "Robot stopped"

			time.sleep(1)

			# robot has stopped, now back up until the object is gone			
			while (self.emergency == True):
				self.Backward(speed_x)

			print "Object is gone, stopping"

			self.Stop()		

			print "Robot stopped, turning"

			time.sleep(2)

			# object is gone, now turn
			for x in range (0,3):
				self.TurnLeft(speed_x)
				time.sleep(1)

			time.sleep(3)

			print "done turning"

			self.Stop()		

if __name__ == '__main__':
	demo1 = Demo1()
	demo1.Run(0.2)
	rospy.spin()
