#!/usr/bin/env python

import rospy,tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

wheelRadius = 3.8 #cm
wheelDistance = 30.5 #cm

#drive to a goal subscribed as /move_base_simple/goal

def makeVelMsg (linearVelocity, angularVelocity):
	global pub
	msg = Twist()
	msg.linear.x = linearVelocity
	msg.angular.z = angularVelocity
	pub.publish(msg)


def navToPose(goal):
	print "spin!"
	rotate(math.atan(goal.position.y/goal.position.x))
	print "move!"
	driveStraight(.2, math.sqrt(math.pow(goal.position.y,2) + math.pow(goal.position.x,2)))
	print "spin!"
	rotate(goal.position.theta)
	print "done!"



def executeTrajectory():
	driveStraight(.2, 1)
	rotate(-math.pi/2)
	driveStraight(.2, 1) 
	rotate(3*math.pi/4)

def spinWheels(u1, u2, time):
	#calculate linear and angular velocity
	#v = (wheelDistance/2) * (u1 + u2)/(u2-u1)
	v = (u1 + u2)/2
	w =  (u1 - u2)/wheelDistance

	print v
	initTime = rospy.Time.now().secs
	
	#simple while loop to move for a set amount of time
	while(rospy.Time.now().secs - initTime <= time):
		#movement message
		makeVelMsg(v,w)

	#stop message
	makeVelMsg(0,0)

def driveStraight(speed, distance):
	global pose

	initx = pose.position.x
	inity = pose.position.y
	
	distanceGone = 0
	
	wentFullDistance = False

	while (not wentFullDistance):
		currentX = pose.position.x
		currentY = pose.position.y
		#calculate how far the robot has moved since the last loop	
		distanceGone = math.sqrt(math.pow((currentX-initx),2)+math.pow((currentY-inity),2))
		print distanceGone
		#if we have gone the desired distance
		if (distanceGone >= distance):
			wentFullDistance = True
			makeVelMsg(0,0)
		else:
			makeVelMsg(speed,0)
			rospy.sleep(0.15)
	
def rotate(angle):
	global pose 

	initAngle = pose.orientation.z
	
	desiredAngle = initAngle + angle
	
	#since the angle of the robot is measured in degrees from -180 to 180, we must put bigger angles inside our range
	if(desiredAngle > 180):
		desiredAngle = desiredAngle -360
	elif(desiredAngle < -180):
		desiredAngle = desiredAngle + 360
		wentFullAngle = False

	while (not wentFullAngle):
		#get the current angle
		currentTheta = pose.orientation.z
		# print currentTheta * 180/math.pi
		#if we want to turn ccw
		if(desiredAngle > 0):
			if(currentTheta >= desiredAngle):
				wentFullAngle = True
				makeVelMsg(0,0)
			else:
				makeVelMsg(0,math.pi/4)
		#if we want to turn cw
		else:
			if(currentTheta <= desiredAngle):
				wentFullAngle = True
				makeVelMsg(0,0)
			else:
				makeVelMsg(0,-math.pi/4)
			
			rospy.sleep(0.15)



		

def driveArc(radius, speed, angle):
	global pose

	#angular velocity of the robot making the arc
	omega = speed/radius

	angleToGo = math.degrees(angle) + math.degrees(pose.orientation.z)

	while((abs(angleToGo) >= 2)):
		makeVelMsg(speed, omega)
		angleToGo = angle + math.degrees(pose.orientation.z)

	makeVelMsg(0,0)

def readBumper(msg):
	if (msg.state == 1):
		print "I hit a wall"
		executeTrajectory()

def timerCallback(event):
	global pose
	global theta
	pose = Pose()
	#from megan's powerpoint	
	#odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(10.0))
	(position, orientation) = odom_list.lookupTransform ('odom', 'base_footprint', rospy.Time(0))

	pose.position.x = position [0] 
	pose.position.y = position [1]
	pose.position.z = position [2]

	odomW = orientation
	q = [odomW[0], odomW[1],odomW[2], odomW[3]]
	roll, pitch, yaw = euler_from_quaternion(q)
	pose.orientation.z = yaw
	theta = math.degrees(yaw)




if __name__ == '__main__':
	rospy.init_node('thagen_lab2_node')
	global pub
	global pose
	global odom_tf
	global odom_list

	pose = Pose()

	pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, None, queue_size=10)
	bumper_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, readBumper, queue_size=1)
	goal_sub = rospy.Subscriber('move_base_simple/goal',PoseStamped, navToPose, queue_size=1)
		odom_list = tf.TransformListener()

	rospy.sleep(rospy.Duration(1,0))
	print "Starting Lab 2"

	rospy.Timer(rospy.Duration(0.01),timerCallback)
	#a while loop that does nothing so we can hit the bumper
	#while(not rospy.is_shutdown()):
		#x = 5
	rospy.sleep(1)

	#Functions for signoffs

	#spinWheels(.2,.9,2)
	#driveStraight(.5,.5)
	#rotate(math.pi/2)
	#executeTrajectory()
	driveArc(0.5,0.1,7)
