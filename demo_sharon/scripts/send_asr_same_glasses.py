#! /usr/bin/env python3
from std_msgs.msg import String, Empty, Bool
import rospy
from sharon_msgs.msg import GlassesData
from array import *

class ASRFromGlasses():
    def __init__(self):
        rospy.loginfo("Initalizing send_asr_same_glasses...")
        self.pubAsr = rospy.Publisher('asr_node/data', String,queue_size=20)
        self.subGlassesData = rospy.Subscriber("comms_glasses_server/data", GlassesData, self.callback)
        self.threshold_execute_trajectory = rospy.get_param('demo_sharon/threshold_execute_trajectory')
        
    def callback(self,data):    
  
        decision = list(data.decision_vector)
        print(decision)
        decision.sort()
        if decision[-1]>=self.threshold_execute_trajectory:
            rospy.sleep(0.2)
            self.pubAsr.publish(data.category)
        
    


if __name__ == '__main__':
    rospy.init_node('send_asr_same_glasses')
    asrGlasses = ASRFromGlasses()
    rospy.spin()

