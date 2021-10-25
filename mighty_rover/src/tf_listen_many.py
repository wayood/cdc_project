#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

def Main():

    # ROS ノードの初期化処理
    rospy.init_node('tf2_listen')

    # リスナーの登録
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    pub_x = rospy.Publisher('/lm_position_x',Float32MultiArray,queue_size = 10)
    pub_y = rospy.Publisher('/lm_position_y',Float32MultiArray,queue_size = 10)
    # 10 Hz
    rate = rospy.Rate(1.0)
    x = Float32MultiArray()
    y = Float32MultiArray()
    for i in range(5):
        x.data.append(i)
        y.data.append(i)
    while not rospy.is_shutdown():
        
        # /map に対する /object_ の Transform を取得
        try:
            t = tfBuffer.lookup_transform('object_133', 'map', rospy.Time())
            #t = tfBuffer.lookup_transform('mug', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rate.sleep()
            continue
        
        if not t:
            continue
        else:

            for i in range(5):
                x.data[i] = t.transform.translation.x
                y.data[i] = t.transform.translation.y
                print(x)
                print(y)
            pub_x.publish(x)
            pub_y.publish(y)
            
        print('{0:.2f}, {1:.2f}, {2:.2f}'.format(
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ))
        print('{0:.2f}, {1:.2f}, {2:.2f}, {3:.2f}'.format(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ))    
        rate.sleep()


if __name__ == '__main__':
    Main()