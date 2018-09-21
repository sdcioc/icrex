#! /usr/bin/python
import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg

## face detection libraries
import cv2
import dlib
import cv_bridge
import math
import json


class FaceDetecttorLogicManager:
	#constructor
    def __init__(self):
        self.rate = rospy.Rate(10);
        self.face_detect_pub = rospy.Publisher(
                    '/experiment/facesdetected',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        self.cvBridge = cv_bridge.CvBridge();
        self.faceDectector = dlib.get_frontal_face_detector();
        rospy.sleep(3);
        rospy.Subscriber("xtion/rgb/image_rect_color", std_msgs.msg.String, self.image_subscriber_callback);
        rospy.sleep(3);
        ## face_detec

    def rect_to_bb(self, rect):
        x = rect.left();
        y = rect.top();
        w = rect.right() - x;
        h = rect.bottom() - y;
        return (x, y, w, h);

    def image_subscriber_callback(self, reply):
        rospy.loginfo("[image_subscriber_callback] message format for scan {}".format(reply));
        frame = self.cvBridge.imgmsg_to_cv2(reply, 'bgr8');
        faces_detected = self.faceDectector(frame, 1);
        rospy.loginfo("[image_subscriber_callback] people detected {}".format(len(faces_detected)));
        to_publish = {};
        to_publish['faces'] = faces_detected;
        to_publish['time'] = rospy.get_time();
        self.face_detect_pub.publish(json.dumps(to_publish));
        for (i, rect) in enumerate(faces_detected):
            (x, y, w, h) = self.rect_to_bb(rect);
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2);
            cv2.putText(frame, "Face #{}".format(i + 1), (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2);
        cv2.imshow("Output", frame);

    def check_door_open(self):
        try:
            reply = rospy.wait_for_message(
            '/scan',
            sensor_msgs.msg.LaserScan, 3);
        except rospy.exceptions.ROSException:
            rospy.loginfo("[ERROR][check_door_open] No Info from /scan");
            return False;
        ranges = reply.ranges;
        rospy.loginfo("[check_door_open] message format for scan {}".format(reply));
        mid_index = len(reply.ranges);
        new_ranges = [x for x in ranges[mid_index-5:mid_index+5] if not math.isnan(x)];
        average_dist = sum(new_ranges)/len(new_ranges);
        if(average_dist > 0.4):
            return True;
        else:
            return False;

    def check_number_people(self):
        try:
            reply = rospy.wait_for_message(
            '/xtion/rgb/image_rect_color',
            sensor_msgs.msg.Image, 3);
        except rospy.exceptions.ROSException:
            rospy.loginfo("[ERROR][check_number_people] No Info from /xtion/rgb/image_rect_color");
            return -1;
        rospy.loginfo("[check_number_people] message format for scan {}".format(reply));
        frame = self.cvBridge.imgmsg_to_cv2(reply, 'bgr8');
        faces_detected = self.faceDectector(frame, 1);
        rospy.loginfo("[check_number_people] people detected {}".format(len(faces_detected)));
        return len(faces_detected);

if __name__ == '__main__':
    rospy.init_node('experiment_face_detect_node', anonymous=True);
    #test_POI_classes()
    try:
        rospy.loginfo("[EXPERIMENT__FACE_DETECT_NODE] STARTED");
        my_logic_manager = FaceDetecttorLogicManager();
        while(True):
            n = my_logic_manager.check_number_people();
            rospy.sleep(3);
            if(n == -1):
                break;
        rospy.spin();
    except KeyboardInterrupt:
        pass;