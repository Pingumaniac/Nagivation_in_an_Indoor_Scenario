#!/usr/bin/env python2
"""
 The explore node causes the robot to explore the environment autonomously while mapping the world
 SUBSCRIBERS:
  sub_map (nav_msgs/OccupancyGrid) - represents a 2-D grid map, in which each cell represents the probability of occupancy.
"""

import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction, MoveBaseActionFeedback
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from random import randrange
import time
import sys
import math
from matplotlib import pyplot as plt
#from sensor_msgs import LaserScan

class Point:
    def __init__(self, x, y, compare):
	self.x = x
	self.y = y
	#self.cut = cut
	self.left = -1
	self.right = -1
	self.compare = compare

class Explore:

    def __init__(self):
        """ Initialize environment
        """
        # Initialize rate:
        self.rate = rospy.Rate(1)

        # Simple Action Client:
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))
        rospy.logdebug("move_base is ready") 

        self.x = 0
        self.y = 0
        self.completion = 0

	self.rrt = []
	self.rrt_ready = True
	self.data = None

        # Initialize subscribers:
        self.map = OccupancyGrid()
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        #self.sub_pos = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.pose_callback)
	#self.sub_pos = rospy.Subscriber('/base_link', MoveBaseActionFeedback, self.pose_callback)
	self.count = 0

	self.rrt.append(Point(210, 128, 'x'))
        #self.sub_laser = rospy.Subscriber('/scan', OccupancyGrid, self.cost_callback)
	rospy.logdebug("PUB")
	self.pub = rospy.Publisher('rrt', OccupancyGrid, queue_size=10)
	rospy.logdebug("PUBLISHED")

        time.sleep(8)

    def cost_callback(self, msg):
	rospy.logdebug("cost-map")
	rospy.logdebug(msg.info.width)
	return True

    def build_rrt(self):
	while not self.is_frontier(self.rrt[len(self.rrt)-1]):
	    #rospy.logdebug("RRRRTTTT")
	    rand_x = randrange(0, 385)
	    rand_y = randrange(0, 385)
	    closest = self.closest(rand_x, rand_y)
	    
	    close_x = self.rrt[closest].x
	    close_y = self.rrt[closest].y
	    gap_x = rand_x - close_x
	    gap_y = rand_y - close_y
	    dist = math.sqrt(gap_x*gap_x + gap_y*gap_y)

	    if dist > 4:
		gap_x /= dist
		gap_y /= dist
		gap_x *= randrange(4,10)
		gap_y *= randrange(4,10)
	    new_x = round(close_x + gap_x)
	    new_y = round(close_y + gap_y)
	    #rospy.logdebug(str(new_x) + " " + str(new_y))
	    #rospy.logdebug(self.map_position(new_x, new_y))
	    if self.valid_path(new_x, new_y, close_x, close_y):
	    	rospy.logdebug("VALIDITYYY")
		rospy.logdebug(len(self.rrt))
		rospy.logdebug(str(new_x) + " " + str(new_y))
		if self.rrt[closest].compare == 'x':
		    if new_x < close_x:
			self.rrt[closest].left = len(self.rrt)
		    else:
			self.rrt[closest].right = len(self.rrt)
		    self.rrt.append(Point(int(new_x), int(new_y), 'y'))
		else:
		    if new_y < close_y:
			self.rrt[closest].left = len(self.rrt)
		    else:
			self.rrt[closest].right = len(self.rrt)
		    self.rrt.append(Point(int(new_x), int(new_y), 'x'))

		out = OccupancyGrid()
		self.arr2 = [0 for _ in range(384*384)]
		for i in range(len(self.rrt)):
	    	    self.arr2[self.rrt[i].x*384+self.rrt[i].y] = 50
	    	self.fill(close_x, close_y)
	    	self.fill(new_x, new_y)
	    	self.fill(rand_x, rand_y)
		out.data = self.arr2
		#plt.imshow(self.arr2, interpolation='nearest')
		#plt.show()
		out.info = self.data.info
		self.pub.publish(out)
		time.sleep(0.2)

	arr = [[0 for _ in range(384)] for _ in range(384)]
	diffs = []
	for i in range(384):
	    for j in range(384):
		arr[i][j] = self.map_position(i, j)
		if arr[i][j] not in diffs:
		     diffs.append(arr[i][j])
		if arr[i][j] == -1:
		    arr[i][j] = 50
		elif arr[i][j] == 0:
		    arr[i][j] = 100
		elif arr[i][j] == 100:
		    arr[i][j] = 150
	rospy.logdebug("POOOPOPOPOOOOOOO")
	rospy.logdebug(diffs)
	
	out = OccupancyGrid()
	self.arr2 = [0 for _ in range(384*384)]
	for i in range(len(self.rrt)):
	    arr[self.rrt[i].x][self.rrt[i].y] = 255
	    self.arr2[self.rrt[i].x*384+self.rrt[i].y] = 50
	out.data = self.arr2
	#plt.imshow(self.arr2, interpolation='nearest')
	#plt.show()
	out.info = self.data.info
	self.pub.publish(out)

	self.rrt_ready = False
	self.set_goal()

	
    def fill(self, x, y):
	for i in range(-2,3):
	    for j in range(-2,3):
		self.arr2[int(x+i)*384+int(y+j)] = 100

    def valid_path(self, new_x, new_y, close_x, close_y):
	for i in range(-2,3):
	    for j in range(-2,3):
		if self.map_position(new_x+i, new_y+j) >= 80:
		    return False
		elif self.map_position(new_x+i, new_y+j) == -1:
		    return False
	for i in range(-2,3):
	    for j in range(-2,3):
		if self.map_position(close_x+i, close_y+j) >= 80:
		    return False
	mid_x = int((new_x + close_x)/2)
	mid_y = int((new_y + close_y)/2)
	for i in range(-2,3):
	    for j in range(-2,3):
		if self.map_position(mid_x+i, mid_y+j) >= 80:
		    return False
	q3_x = int((new_x + mid_x)/2)
	q3_y = int((new_y + mid_y)/2)
	for i in range(-2,3):
	    for j in range(-2,3):
		if self.map_position(q3_x+i, q3_y+j) >= 80:
		    return False
	q1_x = int((mid_x + close_x)/2)
	q1_y = int((mid_y + close_y)/2)
	for i in range(-2,3):
	    for j in range(-2,3):
		if self.map_position(q1_x+i, q1_y+j) >= 80:
		    return False
	return True
	    

    def closest(self, x, y):
	cur_point = 0
	cur_compare = 'x'
	queue = []
	while True:
	    if cur_compare == 'x':
		cur_compare = 'y'
		if x < self.rrt[cur_point].x:
		    if self.rrt[cur_point].left == -1:
			break
		    else:
			cur_point = self.rrt[cur_point].left
		else:
		    if self.rrt[cur_point].right == -1:
			break
		    else:
			cur_point = self.rrt[cur_point].right
	    else:
		cur_compare = 'x'
		if y < self.rrt[cur_point].y:
		    if self.rrt[cur_point].left == -1:
			break
		    else:
			cur_point = self.rrt[cur_point].left
		else:
		    if self.rrt[cur_point].right == -1:
			break
		    else:
			cur_point = self.rrt[cur_point].right

	return cur_point

    def is_frontier(self, point):
	if self.map_position(point.x, point.y) == -1:
	    return False

	unknown_within = False
	for i in range(-4,5):
	    for j in range(-4,5):
		if self.map_position(point.x+i, point.y+j) >= 80:
		    return False
		if self.map_position(point.x+i, point.y+j) == -1:
		    if abs(i) <= 2 or abs(j) <= 2:
			return False
		    unknown_within = True
	
	return unknown_within

    def pose_callback(self, msg):
	rospy.logdebug("POSEEEEEEEEEEE")
	self.x = msg.feedback.base_position.pose.position.x
	self.y = msg.feedback.base_position.pose.position.y
	rospy.logdebug(self.x)
	rospy.logdebug(self.y)

    def map_position(self, x, y):
	#rospy.logdebug("MAPPOSITION")
	#rospy.logdebug(round(x*self.data.info.width + y))
	return self.data.data[int(x*self.data.info.width + y)]

    def map_callback(self, data):
        """ Callback function for map subscriber.
        Subscribes to /map to get the OccupancyGrid of the map.
        """
        valid = False

	time.sleep(3)

        while valid is False:
            map_size = randrange(len(data.data))
            self.map = data.data[map_size]

            edges = self.check_neighbors(data, map_size)
            if self.map != -1 and self.map <= 20 and edges is True:
                valid = True
        
	self.data = data
	
        row = map_size / 384
        col = map_size % 384

	if self.rrt_ready:
	    rospy.logdebug("DATAAAAAA")
	    rospy.logdebug(data.info.origin)
	    self.build_rrt()
    

    def set_goal(self):
        """ Set goal position for move_base.
        """
        rospy.logdebug("Setting goal")

        # Create goal:
        goal = MoveBaseGoal()

        # Set random goal:
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.rrt[len(self.rrt)-1].y*0.05-10
        goal.target_pose.pose.position.y = self.rrt[len(self.rrt)-1].x*0.05-10
        goal.target_pose.pose.orientation.w = 1.0
        #rospy.logdebug(f"goal: {goal.target_pose.pose.position.x, goal.target_pose.pose.position.y}")
        self.move_base.send_goal(goal, self.goal_status)


    def goal_status(self, status, result):
        """ Check the status of a goal - goal reached, aborted,
        or rejected.
        """
        self.completion += 1

        # Goal reached
        if status == 3:
            rospy.loginfo("Goal succeeded")
	    self.rrt_ready = True

        # Goal aborted
        if status == 4:
            rospy.loginfo("Goal aborted")
	    self.rrt_ready = True

        # Goal rejected
        if status == 5:
            rospy.loginfo("Goal rejected")
	    self.rrt_ready = True


    def check_neighbors(self, data, map_size):
        """ Checks neighbors for random points on the map.
        """
        unknowns = 0
        obstacles = 0

        for x in range(-3, 4):
            for y in range(-3, 4):
                row = x * 384 + y
                try:
                    if data.data[map_size + row] == -1:
                        unknowns += 1
                    elif data.data[map_size + row] > 0.65:
                        obstacles += 1
                except IndexError:
                    pass
        if unknowns > 0 and obstacles < 2:
            return True
        else:
            return False


def main():
    """ The main() function """
    rospy.init_node('explore', log_level=rospy.DEBUG)
    Explore()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException