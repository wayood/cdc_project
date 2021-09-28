#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
import cv2
#from BoundingBox.msg import BoundingBoxes
#from BoundingBoxes.msg import BoundingBox
 
################################################################
# 
# Draw all detection box Start
# 
################################################################
while True:
    
    Boxes = []
    scores=[]
    id_count = 0
    
    # Loop over all detections and draw detection box if confidence is above minimum threshold
    for i in range(len(scores)):
        if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):
    
            # Get bounding box coordinates and draw box
            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
            ymin = int(max(1,(boxes[i][0] * imH)))
            xmin = int(max(1,(boxes[i][1] * imW)))
            ymax = int(min(imH,(boxes[i][2] * imH)))
            xmax = int(min(imW,(boxes[i][3] * imW)))
            
            cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)
    
            # Draw label
            object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
            label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
    
            boundingbox = BoundingBox(xmin = xmin,xmax = xmax,ymin = ymin,ymax = ymax,id = id_count,Class = object_name,probability = scores[i])
            Boxes.append(boundingbox)
            id_count = id_count + 1
            
    ################################################################
    # 
    # BoundingBox Publish Start
    # 
    ################################################################
    
    array_forPublish = BoundingBoxes(bounding_boxes=Boxes)
    motor.publish(array_forPublish)
    
    # All the results have been drawn on the frame, so it's time to display it.
    cv2.imshow('Object detector', frame)
 
    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break
 
# Clean up
cv2.destroyAllWindows()
cap.release()