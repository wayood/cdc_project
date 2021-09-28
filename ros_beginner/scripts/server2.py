#!/usr/bin/env python

import rospy

from ros_beginner.srv import WordCount,WordCountResponse


def count_words(request):
    rospy.loginfo('called!')
    rospy.loginfo(request)
    rospy.loginfo(request.words)
    rospy.loginfo(request.words.split())
    rospy.loginfo(len(request.words.split()))

    return WordCountResponse(len(request.words.split()))


rospy.init_node('service_server')

service = rospy.Service('word_count', WordCount, count_words)

#print(service)
#rospy.loginfo(service)
rospy.spin()
