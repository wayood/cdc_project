#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros

def Main():

    # ROS ノードの初期化処理
    rospy.init_node('tf2_listener')

    # リスナーの登録
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # 10 Hz
    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():

        # /map に対する /object_ の Transform を取得
        try:
            t = tfBuffer.lookup_transform('move_base_simple/goal', 'base_link', rospy.Time())
            #t = tfBuffer.lookup_transform('mug', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rate.sleep()
            continue

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