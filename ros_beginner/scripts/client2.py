#!/usr/bin/env python

import rospy

from ros_beginner.srv import WordCount

import sys


rospy.init_node('service_client')

rospy.wait_for_service('word_count')

#print(words)
#print words, '->', word_count.count

word_counter = rospy.ServiceProxy('word_count', WordCount) #server->client

words = ' '.join(sys.argv[1:]) #->request

word_count = word_counter(words)
print(words)
#print('word[0] = ' + words[0])
#print('word[1] = ' + words[1])
#print('word[2] = ' + words[2])


print words, '->', word_count.count

print '->',word_count
print '->',word_count.count

print(word_count)

