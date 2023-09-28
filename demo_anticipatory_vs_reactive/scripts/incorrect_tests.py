#! /usr/bin/env python3

import os
import datetime
import numpy as np
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String, Empty, Bool
from sharon_msgs.msg import GlassesData
import sys

word_dict = ['sugar',
             'cereals', 
             'coffee',
             'bread',
             'milk',
             'water',
             'butter',
             'nesquick', 
             'please',
             ]

category_gaze = 'water'
category_asr = 'cereals'

decision = 0.7
freq = 10.0
mu = 10.73
sigma = 7.9
asr_time_from_gaze = 0
while asr_time_from_gaze < 2:
    asr_time_from_gaze = np.random.normal(mu, sigma, 1)

print('asr_time_from_gaze: ', asr_time_from_gaze)
if __name__ == '__main__':
    rospy.init_node('incorrect_tests_node')
    pubAsr = rospy.Publisher('asr_node/data', String,queue_size=20)
    pubGlassesData = rospy.Publisher('comms_glasses_server/data', GlassesData,queue_size=20)
    print(sys.path[0])
    categories = np.load(sys.path[0]+'/'+'categories.npy')

    glassesData = GlassesData()
    glassesData.header.stamp = rospy.Time.now()
    glassesData.decision_vector = [0]*len(categories)

    glassesData.category = category_gaze
    index = np.where(categories == category_gaze)[0][0]
    glassesData.decision_vector[index] = decision
    pubGlassesData.publish(glassesData)


    print("Glasses data sent: ", glassesData.category, glassesData.decision_vector)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        pubGlassesData.publish(glassesData)
        if(rospy.Time.now() - start_time).to_sec() > asr_time_from_gaze:
            msgText = String()
            msgText.data = category_asr
            print("ASR data sent: ", msgText.data)
            pubAsr.publish(msgText)
            break
        rospy.sleep(1/freq)
    
