#!/usr/bin/env python
import time
import rospy
import threading
import std_srvs.srv
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

PI = 3.1415926535897
LINEAR_SPEED = 1
ANGULAR_SPEED = 10

def _get_vel_msg():
	vel_msg = Twist()
	vel_msg.linear.x=0
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	return vel_msg
#end def

class Robot:

	def __init__(self, name='robot', turtle='turtle1'):
		#Node, publisher, and subscriber
		rospy.init_node('robot', anonymous=True)
		rospy.wait_for_service('clear')
		sim_reset = rospy.ServiceProxy('reset', std_srvs.srv.Empty)
		sim_reset()
		self.velocity_publisher = rospy.Publisher('/{}/cmd_vel'.format(turtle), Twist, queue_size=10)
		self.pose_subscriber = rospy.Subscriber('/{}/pose'.format(turtle), Pose, self._pose_callback)
		self._rate = 100
		self.rate = rospy.Rate(self._rate)
		# Attributes
		self.pose = Pose()
		self.speed = Twist()
		# Async
		self._thread = None
		self._async = False
	#end def

	def __del__(self):
		self._async = False
		self._thread.join(100)
	#end def

	@property
	def angular(self):
		return self.speed.angular.z
	@angular.setter
	def angular(self, value):
		self.speed.angular.z = value
	#end def

	@property
	def linear(self):
		return self.speed.linear.x
	@linear.setter
	def linear(self, value):
		self.speed.linear.x = value
	#end def

	@property
	def x(self):
		return self.pose.x
	#end def

	@property
	def y(self):
		return self.pose.y
	#end def

	@property
	def theta(self):
		return self.pose.theta
	#end def
		

	def _start_async(self):
		if self._thread != None:
			return
		self._thread = threading.Thread(target=self._async_publish)
		self._async = True
		self._thread.start()
	#end def
	
	def _join_async(self):
		if self._thread == None:
			return
		self._async = False
		self._thread.join(1000)
		self._thread = None
	#end def

	def _async_publish(self):
		while(self._async):
			self.velocity_publisher.publish(self.speed)
			self.rate.sleep()
	#end def

	def _pose_callback(self, pose):
		self.pose = pose
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)
		# print '\tturtle: ({:0.4f}, {:0.4f}, {:0.4f})'.format(self.pose.x, self.pose.y, self.pose.theta)
	#end def


	def stop(self):
		self._join_async()
		self.speed.linear.x=0
		self.speed.linear.y=0
		self.speed.linear.z=0
		self.speed.angular.x = 0
		self.speed.angular.y = 0
		self.speed.angular.z = 0

		self.velocity_publisher.publish(self.speed)
	#end def


	def advance(self, speed=LINEAR_SPEED):
		self.speed.angular.z = 0
		self.speed.linear.x = speed
		# self.velocity_publisher.publish(self.speed)
		self._start_async()
	#end def

	def spin(self, speed=ANGULAR_SPEED):
		#Converting from degrees to radians
		speed = 10*speed*PI/180.0
		self.speed.angular.z = speed
		self.speed.linear.x = 0
		# self.velocity_publisher.publish(self.speed)
		self._start_async()
	#end def
		
	def turn(self, speed=ANGULAR_SPEED):
		self.rotate(180, speed)
	#end def

	def rotate(self, degrees, speed=ANGULAR_SPEED):
		self._join_async()
		#Converting from degrees to radians
		speed = 10*speed*PI/180.0
		angle = self.pose.theta + degrees*PI/180.0
		step = abs(speed * 1.0/self._rate)

		diff = angle - self.pose.theta
		self.speed.angular.z = math.copysign(speed, diff)
		i = 0
		while(abs(diff) > 0.01 ):
			if i > 200: return
			i+=1
			diff = angle - self.pose.theta
			if abs(diff) < step:
				self.speed.angular.z = speed * diff/step
			self.velocity_publisher.publish(self.speed)
			self.rate.sleep()
		self.stop()
	#end def

	def step(self):
		self._join_async()
		self.speed.linear.x = LINEAR_SPEED
		self.velocity_publisher.publish(self.speed)
		for i in range(1, self._rate):
			self.rate.sleep()
		self.stop()
	#end def

	def circle(self, linear=LINEAR_SPEED, angular=ANGULAR_SPEED):
		self.speed.linear.x = linear
		self.speed.angular.z = 10*angular*PI/180.0
		self.velocity_publisher.publish(self.speed)
		self._start_async()
	#end def

	def idle(self):
		self.rate.sleep()
	#end def

#end class


if __name__ == '__main__':
	robot = Robot()
	robot.rotate(90)
	robot.rotate(-90)
	robot.turn()
	robot.step()
	robot.spin()
	time.sleep(5)
	robot.circle()
	time.sleep(5)
	robot.advance()
	time.sleep(5)
	robot.stop()
	
#end def