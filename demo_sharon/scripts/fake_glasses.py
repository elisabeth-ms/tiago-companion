#! /usr/bin/env python3

import rospy
import numpy as np
import sys
from sharon_msgs.msg import GlassesData

freq = 6.0
number_categories = 1
category0 = 'background'
category1 = 'cereals'
decision = 0.0
category2 = 'milk'

execute_plan_threshold = 0.82
max_decision = 1.0
count_max_decision = 30
decision_background = 0.4
count_background = 0
global_count = 0
if __name__ == '__main__':
    rospy.init_node('fake_glasses')
    print(sys.path[0])
    categories = np.load(sys.path[0]+'/'+'categories.npy')
    print(categories)
    
    pubGlassesData = rospy.Publisher('comms_glasses_server/data', GlassesData,queue_size=20)


    if number_categories == 1:
        count = 0
        up = True
        while not rospy.is_shutdown():
            glassesData = GlassesData()
            glassesData.header.stamp = rospy.Time.now()
            glassesData.decision_vector = [0]*len(categories)
            
            if up:
                if count_background < 50:
                    decision_background = decision_background + 1/(30.0*freq)
                    count_background += 1
                elif count_background == 50:
                    up = False
                    count_background = 0
            else:
                if count_background < 50:
                    decision_background = decision_background - 1/(30.0*freq)
                    count_background += 1
                elif count_background == 50:
                    up = True
                    count_background = 0
                    
            if (decision<max_decision and count<count_max_decision):
                decision = decision + 1/(5.0*freq)
            elif (decision>=max_decision and count<count_max_decision):
                decision = max_decision
                count+=1
            elif count>=count_max_decision:
                decision = decision - 1/(5.0*freq)
                if decision<0:
                    decision = 0.0

            if decision> decision_background:
                index = 0
                if global_count <3:
                    glassesData.category = category1
                    index = np.where(categories == category1)[0][0]
                elif global_count >=3 and global_count<6:
                    glassesData.category = category2
                    index = np.where(categories == category2)[0][0]
                

                # index = np.where(categories == category1)[0][0]
                glassesData.decision_vector[index] = decision
            else:
                glassesData.category = category0
                index = np.where(categories == category0)[0][0]
                glassesData.decision_vector[index] = decision_background
            
            pubGlassesData.publish(glassesData)
            
            global_count += 1
            if global_count == 6:
                global_count = 0
            
            
    
                    
                
            rospy.sleep(1/freq)

