#!/usr/bin/env python
"""
    Copyright (C) 2014 Eduardo Feo
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from optitrack_msgs.msg import RigidBodies
from optitrack_msgs.msg import Markers
import lcm
from poselcm import pose_list_t
from poselcm import pose_t
from markerlcm import marker_list_t
from markerlcm import marker_t

import time

import math
#############################################################
#############################################################
class LCMTrackerBridge():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("optitrack_lcm_bridge")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.nodes = {}
        self.markers = []
        self.message_counter = 0
        self.marker_msg_counter = 0
        self.channel = rospy.get_param("~channel", "TRACK")
        self.marker_channel = rospy.get_param("~channel", "markers")
        print "Publishing LCM channel",self.channel
        self.x_shift = rospy.get_param("~x_shift", 0.0)
        self.y_shift = rospy.get_param("~y_shift", 0.0)
    
        rospy.Subscriber('/optitrack/rigid_bodies', \
                         RigidBodies, self.trackCallback)
        rospy.Subscriber('/optitrack/unlabeled_markers', \
                         Markers, self.markerCallback)

        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1") 

        self.publish_rigid_bodies = rospy.get_param("~publish_rigid_bodies",True)
        self.publish_markers = rospy.get_param("~publish_markers",False)
    
        self.rate = rospy.get_param("~rate", 30)
        print "rate = ",self.rate
        #self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        #self.left = 0
        #self.right = 0
        
    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        #idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_start = 0
        #self.ticks_since_target = self.timeout_ticks
    
        ###### main loop  ######
        while not rospy.is_shutdown():
            self.spinOnce()
            r.sleep()
            #print "sleep over"
            self.ticks_since_start +=1
             

    #############################################################

    #############################################################
    def spinOnce(self):
    #############################################################
        tt = self.ticks_since_start * (1.0/self.rate)
        if self.publish_rigid_bodies:
            self.publishRigidBodies()
        if self.publish_markers:
            self.publishMarkers()

    def publishRigidBodies(self):
        # testing
        msg = pose_list_t()
        msg.timestamp = int(time.time() * 1000000)
        msg.poses=[]
        #print len(self.nodes)
        for (r, (pos,ori)) in self.nodes.items():
            pose = pose_t()
            pose.robotid = r
            (x,y,z) = pos
#            pose.position = [math.ceil(x*1000), math.ceil(z*1000), math.ceil(y*1000)]
            pose.position = [math.ceil(-1*x*1000), math.ceil(-1*y*1000), math.ceil(z*1000)]
            (x,y,z,w) = ori
            pose.orientation = [w*10000,x*10000,y*10000,z*10000]
            msg.poses.append(pose)
        msg.n = len(msg.poses)
        self.lc.publish(self.channel, msg.encode())
        if self.ticks_since_start % 10 == 0:
            print "published ", len(msg.poses), "rigid bodies"
        #print "msg published"
    def publishMarkers(self):
        # testing
        msg = marker_list_t()
        msg.timestamp = int(time.time() * 1000000)
        msg.markers=[]
        #print len(self.nodes)
        for (v,pos) in self.markers:
            marker = marker_t()
            marker.visible = 1
            (x,y,z) = pos
#            pose.position = [math.ceil(x*1000), math.ceil(z*1000), math.ceil(y*1000)]
            marker.position = [math.ceil(-1*x*1000), math.ceil(-1*y*1000), math.ceil(z*1000)]
            msg.markers.append(marker)
        msg.n = len(msg.markers)
        if self.ticks_since_start % 10 == 0:
            print "published ",len(msg.markers),"markers"
        self.lc.publish(self.marker_channel, msg.encode())
        #print "msg published"


        
    #############################################################
    def trackCallback(self,msg):
    ############################################################
        self.nodes = {}
        for i in range(len(msg.rigid_bodies)):
            rid = msg.rigid_bodies[i].id
            pos = (msg.rigid_bodies[i].pose.position.x, \
                   msg.rigid_bodies[i].pose.position.y, \
                   msg.rigid_bodies[i].pose.position.z)
            ori = (msg.rigid_bodies[i].pose.orientation.x, \
                   msg.rigid_bodies[i].pose.orientation.y, \
                   msg.rigid_bodies[i].pose.orientation.z, \
                   msg.rigid_bodies[i].pose.orientation.w )
            self.nodes[rid] = (pos, ori)
        self.message_counter += 1
        """ 
        print a message every 100 tracks received 
        to check if the system is alive
        """
        if self.message_counter % 100 == 0:
            print  "rigid_bodies_msg__counter: ", self.message_counter
        #print "got msg ",str(msg)
        #rospy.loginfo("-D- twistCallback: %s" % str(msg))

    #############################################################
    def markerCallback(self,msg):
    ############################################################
        self.markers = []
        for i in range(len(msg.markers)):
            vis = msg.markers[i].visible
            pos = (msg.markers[i].position.x, \
                   msg.markers[i].position.y, \
                   msg.markers[i].position.z)
            self.markers.append((vis,pos))
        self.marker_msg_counter += 1
        """ 
        print a message every 100 tracks received 
        to check if the system is alive
        """
        if self.marker_msg_counter % 100 == 0:
            print  "marker_message_counter: ", self.marker_msg_counter
        #print "got msg ",str(msg)
        #rospy.loginfo("-D- twistCallback: %s" % str(msg))
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    mybridge = LCMTrackerBridge()
    mybridge.spin()
