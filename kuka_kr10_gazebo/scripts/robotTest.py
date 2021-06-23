#!/usr/bin/env python

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Empty
from std_msgs.msg import Header

class PublishPoint():
	def __init__(self):
		self.trajPub = rospy.Publisher('/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=10)

		self.msg2pub = FollowJointTrajectoryActionGoal()



		


	def publish_msg(self):
		# Set the message to publish as command.
		traj_vector = FollowJointTrajectoryActionGoal()
		# Current ROS time stamp
		h = Header()
		h.stamp = rospy.Time.now()
		traj_vector.header = h
		traj_vector.goal.trajectory.joint_names.append('joint_a1');
		traj_vector.goal.trajectory.joint_names.append('joint_a2');
		traj_vector.goal.trajectory.joint_names.append('joint_a3');
		traj_vector.goal.trajectory.joint_names.append('joint_a4');
		traj_vector.goal.trajectory.joint_names.append('joint_a5');
		traj_vector.goal.trajectory.joint_names.append('joint_a6');
		traj_vector.goal_id.stamp = h.stamp
		traj_vector.goal_id.id = 'rosPathPlanner' + str(h.stamp.secs)
		h2 = Header()
		h2.stamp.secs = h.stamp.secs #+1
		h2.stamp.nsecs = h.stamp.nsecs #+ 3e8
		traj_vector.goal.trajectory.header.stamp = h2.stamp

		points2pub = [[0.0, -0.5, 0.5, 0.5, 0.0, 0.0, 0.0]]

		iter = 0.0
		for column in points2pub:
			point = JointTrajectoryPoint()
			iter +=1
			for q in column:
				point.positions.append(q)
				point.time_from_start.nsecs = 0
				point.time_from_start.secs = iter*1
			traj_vector.goal.trajectory.points.append(point)
	
 

		print traj_vector
		print 'All systems go!'
		self.trajPub.publish(traj_vector)	

	def run(self):

		

		while not rospy.is_shutdown():

			self.publish_msg()
			print "Running!"
			rospy.sleep(2)


if __name__ == '__main__':

	rospy.init_node('RobotTest')
	node = PublishPoint()
	node.run()
