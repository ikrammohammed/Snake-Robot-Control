#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

class Joint:
	def __init__(self, ID):
		self.goal_pos = 0.0
		self.current_pos = 0.0
		self.currentPosD = 0.0		
		self.error = 0.0
		self.velocity = 0.0
		self.load = 0.0
		self.isMoving = True
		self.currentAngle = 0.0	
		self.reachedAngle = False
		self.goalPos = 0.0 
		self.currentPos =0.0
		self.error = 0.0
		self.velocity = 0.0
		self.load = 0.0
		self.isMoving = 0.0	

		self.topic = '/joint'+str(ID)+'_controller/command'
		self.pub = rospy.Publisher(self.topic, Float64, queue_size=10)
		self.sub = rospy.Subscriber('/joint'+str(ID)+'_controller/state', JointState, self.callback)
	def moveToPos(self, angle):
		angle = angle #* (3.14159265358979/180)
		self.pub.publish(angle)
	def callback(self, data):
		self.goalPos = data.goal_pos 
		self.currentPos = data.current_pos
		self.currentPosD = data.current_pos * (180/3.14159265358979)
		self.error = data.error
		self.velocity = data.velocity
		self.load = data.load
		self.isMoving = data.is_moving
		#convert currentPos into Current angle
		self.currentAngle = self.currentPos  #Fix me, I do not work

def talker():
	joint1 = Joint(1)
	joint2 = Joint(2)
	joint3 = Joint(3)
	joint4 = Joint(4)
	joint5 = Joint(5)
	joint6 = Joint(6)
	joint7 = Joint(7)
	joint8 = Joint(8)
	joint9 = Joint(9)
	joint10 = Joint(10)
	joint11 = Joint(11)
	joint12 = Joint(12)
	joint13 = Joint(13)
	joint14 = Joint(14)
	joint15 = Joint(15)
	joint16 = Joint(16)
	rospy.init_node('jointControllerBadAss', anonymous=True)
	rate = rospy.Rate(10) # 10hz
    
	State = True
	
		
	while not rospy.is_shutdown():
		
		joint1.moveToPos(-0.11)
		joint2.moveToPos(-1.45)
		joint3.moveToPos(0.3)
		joint4.moveToPos(-0.25)
		joint5.moveToPos(-1.44)
		joint6.moveToPos(1.5)
		joint7.moveToPos(0.0)
		joint8.moveToPos(-1.27)
		joint9.moveToPos(0.0)
		joint10.moveToPos(1.34)
		joint11.moveToPos(0.0)
		joint12.moveToPos(1)
		joint13.moveToPos(0)
		joint14.moveToPos(0)
		joint15.moveToPos(0)
		joint16.moveToPos(0)
		
		
		
		print 'Joint1 position = ' + str(joint1.currentPosD)
		print 'Joint2 position = ' + str(joint2.currentPosD)
		print 'Joint3 position = ' + str(joint3.currentPosD)
		print 'Joint4 position = ' + str(joint4.currentPosD)
		print 'Joint5 position = ' + str(joint5.currentPosD)
		print 'Joint6 position = ' + str(joint6.currentPosD)
		print 'Joint7 position = ' + str(joint7.currentPosD)
		print 'Joint8 position = ' + str(joint8.currentPosD)
		print 'Joint9 position = ' + str(joint9.currentPosD)
		print 'Joint10 position = ' + str(joint10.currentPosD)
		print 'Joint11 position = ' + str(joint11.currentPosD)
		print 'Joint12 position = ' + str(joint12.currentPosD)
		print 'Joint13 position = ' + str(joint13.currentPosD)
		print 'Joint14 position = ' + str(joint14.currentPosD)
		print 'Joint15 position = ' + str(joint15.currentPosD)
		print 'Joint16 position = ' + str(joint16.currentPosD)
				

		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
