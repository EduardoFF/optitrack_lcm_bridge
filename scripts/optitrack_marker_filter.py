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
from optitrack_msgs.msg import RigidBodies
from geometry_msgs.msg import PoseStamped
from optitrack_msgs.msg import Markers, Marker
import lcm
from markerlcm import marker_list_t
from markerlcm import marker_t
from sklearn.cluster import KMeans
import numpy as np
import time

import math
#############################################################
#############################################################
class MarkerFilter():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("optitrack_marker_filter")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.nodes = {}
        self.markers = []
        self.message_counter = 0
        self.marker_msg_counter = 0
        self.marker_channel = rospy.get_param("~channel", "markers")
        print "Publishing LCM channel",self.marker_channel
        self.x_shift = rospy.get_param("~x_shift", 0.0)
        self.y_shift = rospy.get_param("~y_shift", 0.0)
    
        rospy.Subscriber('/optitrack/rigid_bodies', \
                         RigidBodies, self.trackCallback)
        rospy.Subscriber('/optitrack/unlabeled_markers', \
                         Markers, self.markerCallback)
        self.f_markers_pub = rospy.Publisher('/filtered_markers', Markers, queue_size=10)


        self.lc_marker = lcm.LCM("udpm://239.255.76.67:7667?ttl=0,transmit_only=true")
        self.do_clustering = False

        self.publish_markers = rospy.get_param("~publish_markers",True)
    
        self.rate = rospy.get_param("~rate", 10)
        self.number_of_tracked_markers = rospy.get_param("~number_of_tracked_markers", 10)
        # if we are getting 120 msgs per second, we cluster every 0.25 seconds
        self.cluster_each_n = rospy.get_param("~cluster_each_n", 30)
        
        print "rate = ",self.rate
        """ threshold in mm for fitering markers - to avoid unlabelled markers that actually
        belong to some rigid body 
        """
        self.marker_rb_threshold = 50
        self.marker_h_threshold  = 100 # in mm
        self.marker_bounding_rect = [[-4,-1.8],[-4,0]]
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
        self.publishRigidBodies()
 #       if self.publish_markers:
 #           self.filterMarkers()
#            self.publishMarkers()

    def publishRigidBodies(self):
        # testing
        #print len(self.nodes)
        """ clear the last published position """
        self.last_published_position = dict()
        for (r, (pos,ori)) in self.nodes.items():
            (x,y,z) = pos
            self.last_published_position[r] = list([math.ceil(-1*x*1000), math.ceil(-1*y*1000), math.ceil(z*1000)])
            
    def dist2d(self, (x1,y1,z1),(x2,y2,z2)):
        return ((x1-x2)**2 + (y1-y2)**2)**0.5
    
    def filterMarkers(self):
        filteredMarkers = list()
        n=len(self.markers)
        for (v,pos) in self.markers:
            if v:
                (x,y,z) = pos
                badboy=False
                for (r,[xr,yr,zr]) in self.last_published_position.items():
                    (xm,ym,zm) = (math.ceil(-1*1000*x), math.ceil(-1*y*1000), math.ceil(z*1000))
                    if self.dist2d( (xm,ym,zm), (xr,yr,zr)) <  self.marker_rb_threshold:
                        badboy=True
                        break
                if not badboy:
                    filteredMarkers.append((v,pos))
        self.markers = filteredMarkers
        if n > len(filteredMarkers):
            print "Filtered markers ",n-len(filteredMarkers)
            
    def publishMarkers(self):
        # testing
        msg = marker_list_t()
        msg.timestamp = int(time.time() * 1000000)
        msg.markers=[]
#        print ""
#        print "going to publish ", len(self.markers), "markers"
#        print ""
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
        self.lc_marker.publish(self.marker_channel, msg.encode())
        #print "msg published"


    #############################################################
    def trackCallback(self,msg):
    ############################################################
        self.nodes = {}
        for i in range(len(msg.rigid_bodies)):
            if msg.rigid_bodies[i].number_of_visible_markers == 0:
                continue
            rid = msg.rigid_bodies[i].id
#            if rid >= 100:
#                continue
                
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


    def clusterMarkers(self, M):
        data = np.array([[x,y,z] for (v,[x,y,z]) in M])
        kmeans = KMeans(n_clusters=self.number_of_tracked_markers)
        kmeans.fit(data)
#        print "clustered ", kmeans.cluster_centers_
        ret = [(1,[x,y,z]) for [x,y,z] in kmeans.cluster_centers_]
        return ret
        
    #############################################################
    def markerCallback(self,msg):
    ############################################################
        #self.markers = []
        [[x1,x2],[y1,y2]] = self.marker_bounding_rect
        for i in range(len(msg.markers)):
            vis = msg.markers[i].visible
            pos = (msg.markers[i].position.x, \
                   msg.markers[i].position.y, \
                   msg.markers[i].position.z)
            if msg.markers[i].position.z > self.marker_h_threshold:
                continue
            if msg.markers[i].position.x < x1 or msg.markers[i].position.x > x2:
                continue
            if msg.markers[i].position.y < y1 or msg.markers[i].position.y > y2:
                continue
            
            
            self.markers.append((vis,pos))
        self.marker_msg_counter += 1
        """ 
        print a message every 100 tracks received 
        to check if the system is alive
        """
        if self.marker_msg_counter % 10 == 0:
            print  "marker_message_counter: ", self.marker_msg_counter, " number of markers", len(self.markers)
        if self.do_clustering:
            if self.marker_msg_counter % self.cluster_each_n == 0:
                self.markers = self.clusterMarkers(self.markers)
                f_msg = Markers()
                for (v,[x,y,z]) in self.markers:
                    marker = Marker()
                    marker.visible = v
                    marker.position.x = x
                    marker.position.y = y
                    marker.position.z = z
                    f_msg.markers.append(marker)
                self.filterMarkers()
                self.f_markers_pub.publish(f_msg)
        else:
            f_msg = Markers()
            for (v,[x,y,z]) in self.markers:
                marker = Marker()
                marker.visible = v
                marker.position.x = x
                marker.position.y = y
                marker.position.z = z
                f_msg.markers.append(marker)
            self.filterMarkers()
            self.f_markers_pub.publish(f_msg)
            self.markers = []

            
                
        #print "got msg ",str(msg)
        #rospy.loginfo("-D- twistCallback: %s" % str(msg))
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    myfilter = MarkerFilter()
    myfilter.spin()
