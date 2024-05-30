#!/usr/bin/env python
#!encoding=utf-8

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.msg import Odometry
import math

class ObstacleAvoidance:
    def __init__(self):
        # 초기화 메시지를 여기서 출력
        rospy.loginfo("Initializing ObstacleAvoidance node")
                
        self.move_cmd = Twist()
        self.OBSTACLE_DISTANCE_THRESHOLD = 0.5  # 장애물 감지 거리 (미터)
        self.CURVE_DISTANCE_FRONT = 1.0  # 오른쪽으로 n 미터
        self.CURVE_DISTANCE_SIDE = 1.0  # 앞쪽으로 m 미터
        
        self.LINEAR_SPEED = 0.2

        self.rate = rospy.Rate(10)
        self.avoiding_obstacle = False
        
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size = 1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.current_theta = 0.
        self.prev_theta = 0.
        self.pose = None

        # step 0: turn left by 30 degrees
        # step 1: move forward by 1 meter
        # step 2: turn right by 30 degrees
        self.avoid_step = -1
        
        rospy.loginfo("ObstacleAvoidance node has been initialized")

    def cbOdom(self, odom_msg):
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)

    def scan_callback(self, data):
        # LIDAR 데이터에서 전방의 최소 거리 계산
        front_distances = data.ranges[len(data.ranges)//3:2*len(data.ranges)//3]
        min_distance = min(front_distances)

        if min_distance < self.OBSTACLE_DISTANCE_THRESHOLD and not self.avoiding_obstacle:
            self.avoiding_obstacle = True
            self.avoid_step = 0
            # set desired theta to current theta + 30 degrees

        if self.avoiding_obstacle:
            self.avoid_obstacle()
        else:
            self.move_in_circle()

    def move_in_circle(self):
        self.move_cmd = Twist()
        self.move_cmd.linear.x = self.LINEAR_SPEED
        self.cmd_vel_pub.publish(self.move_cmd)

    def avoid_obstacle(self):
        # turn left by 30 degrees
        if self.avoid_step == 0:
            if self.delta_t < math.pi/6:
                self.move_cmd = Twist()
                self.move_cmd.angular.z = 0.5
                self.cmd_vel_pub.publish(self.move_cmd)
                delta_t = abs(self.current_theta - self.prev_theta)
                if (delta_t > math.pi):
                    delta_t = 2*math.pi - delta_t
                self.delta_t += delta_t
            else:
                self.avoid_step += 1
                self.delta_t = 0

        # move forward by 1 meter
        # TODO fix this
        elif self.avoid_step == 1:
            if self.pose is not None:
                self.move_cmd = Twist()
                self.move_cmd.linear.x = self.LINEAR_SPEED
                self.cmd_vel_pub.publish(self.move_cmd)
                if self.pose.position.x - self.client.get_result().pose.pose.position.x > self.CURVE_DISTANCE_SIDE:
                    self.avoid_step += 1

        # turn right by 30 degrees
        elif self.avoid_step == 2:
            if self.delta_t < math.pi/6:
                self.move_cmd = Twist()
                self.move_cmd.angular.z = -0.5
                self.cmd_vel_pub.publish(self.move_cmd)
                delta_t = abs(self.current_theta - self.prev_theta)
                if (delta_t > math.pi):
                    delta_t = 2*math.pi - delta_t
                self.delta_t += delta_t
            else:
                self.avoid_step = -1
                self.avoiding_obstacle = False
                self.delta_t = 0


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('obstacle_avoidance_node', anonymous=True)
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass