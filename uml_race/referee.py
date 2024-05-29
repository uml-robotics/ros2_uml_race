#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

max_speed = 5.0

goal_x = -25.0
goal_y = -14.0
goal_e =   2.0

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt
import os

def toS(t):
    return float(t.secs)+float(t.nsecs) / 1000000000.0

def quit(reason):
    print(reason)
    rclpy.shutdown()

def distance(x0, y0, x1, y1):
    dx = x1 - x0
    dy = y1 - y0
    return sqrt(dx*dx + dy*dy)

class Referee(Node):
    def __init__(self):
        super().__init__('Referee')
        self.command_watcher = self.create_subscription(Twist, 'cmd_vel', self.got_cmd_vel,10)
        self.odom_watcher = self.create_subscription(Odometry, 'base_pose_ground_truth', self.got_odom,10)

    def got_odom(self,msg):
        d = distance(msg.pose.pose.position.x, msg.pose.pose.position.y, 
                 goal_x, goal_y)
        if start_time != None and d < goal_e:
            duration = this.get_clock().now() - self.start_time
            quit("Finished in %fs" % toS(duration))

    def got_cmd_vel(self, msg):
        if msg.linear.y > 0 or msg.linear.z > 0:
            quit("Error: Move in bad direction")
        if msg.linear.x > max_speed:
            quit("Error: speed limit exceeded")
        if start_time == None and msg.linear.x != 0:
            self.start_time = this.get_clock().now()
            print( "Start moving at %s" % toS(self.start_time))
        

def main(args=None):
    rclpy.init(args=args)

    referee = Referee()

    rclpy.spin(referee)

    referee.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
