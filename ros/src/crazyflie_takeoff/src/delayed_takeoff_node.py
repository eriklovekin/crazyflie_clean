#!/usr/bin/env python

# Sander Week 2-3 Deliverable: Using sw_hover.launch as model, wait 10 seconds, 
# takeoff, wait 2 seconds, fly to (0,1,2), wait until stable, land.
# See erik_state_plot.py for plotting.

import rospy
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from crazyflie_msgs.msg import PositionVelocityStateStamped

import numpy as np

pub_topic = "/ref"
directions_pub = None #change to use class

def sub_callback(self):
    global executed

    wait2 = float(2)

    rospy.loginfo("Takeoff complete! Performing main flight...")
    
    ## Hovering ##
    rospy.loginfo("Hovering for {} seconds...".format(str(wait2)))

    global executed
    if not executed:

        ## Move to new point ##
        new_point_ = np.array([0,0,20])
        rospy.loginfo("New point : {}".format(str(new_point_)))
        
        reference = PositionVelocityStateStamped()
        reference.header.stamp = rospy.Time.now()
        reference.state.x = new_point_[0]
        reference.state.y = new_point_[1]
        reference.state.z = new_point_[2]
        reference.state.x_dot = 0.0
        reference.state.y_dot = 0.0
        reference.state.z_dot = 0.0
        directions_pub.publish(reference)

        wait3 = float(30)
        rospy.loginfo("Hovering at {} for {} seconds".format(str(new_point_),str(wait3)))
        rospy.sleep(wait3)
        
        ## Land ##
        rospy.loginfo("Landing to begin shortly...")
        land_service = rospy.ServiceProxy('/land',EmptySrv)
        response = land_service()
        rospy.loginfo("Land service called. Response: {}".format(response))
        executed = True
        rospy.loginfo("Maneuver Complete!")

    
def delayed_takeoff_node():
    global directions_pub
    global executed
    executed = False
    wait1 = float(3)

    ## Initialize ##
    rospy.init_node('delayed_takeoff_node', anonymous = True)
    directions_pub = rospy.Publisher(pub_topic,PositionVelocityStateStamped, queue_size=1)
    rospy.Subscriber("/in_flight",EmptyMsg,sub_callback)

    ## Wait on ground ##
    rospy.loginfo("Waiting on ground for {} seconds...".format(str(wait1)))
    rospy.sleep(wait1)

    ## Takeoff ##
    rospy.wait_for_service('/takeoff')
    rospy.wait_for_service('/land')
    takeoff_service = rospy.ServiceProxy('/takeoff',EmptySrv)
    response = takeoff_service()
    rospy.loginfo("Taking off now... Response: {}".format(response))

    rospy.spin()
        

if __name__ == '__main__':
    delayed_takeoff_node()

