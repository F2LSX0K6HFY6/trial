#!/usr/bin/env python

import rospy 
from std_msgs.msg import Float32 
r=0.05
q=2
pre_state=1     #prediction state 
pre_uncer =7    #prediction uncertainty
angel=0
def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)
        angel=data.data
        
def listener():
    
       # In ROS, nodes are uniquely named. If two nodes with the same
       # name are launched, the previous one is kicked off. The
       # anonymous=True flag means that rospy will choose a unique
       # name for our 'listener' node so that multiple listeners can
       # run simultaneously.
       rospy.init_node('listener', anonymous=True)
   
       rospy.Subscriber("yaw_angel", Float32, callback)
   
      # spin() simply keeps python from exiting until this node is stopped
       rospy.spin()
       
def talker():
  pub=rospy.Publisher('filtered_angle',Float32, queue_size=10)
  rospy.init_node('talker',anonymous=True)
  rate= rospy.Rate(10) #10hz
  while not rospy.is_shutdown():
    pre_uncer=pre_uncer+q                                              #prediction uncertanity
   y = angle- pre_state                                                 #residual
    k=pre_uncer/(pre_uncer+r)                                         #kalman factor
    var_state = pre_state +k*y                                       #state variable
    rospy.loginfo(var_state)
    pub.publish(var_state)
    rate.sleep()
   
if __name__ =='__main__':
  try:
     listener()
     talker()
  except rospy.ROSInterruptException:
    pass
