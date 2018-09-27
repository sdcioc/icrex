#! /usr/bin/python
import json

import rospy
import std_msgs.msg
import sensor_msgs.msg

import cv2
import cv_bridge

import rospkg

class PhotoWriter:
    #constructor
    def __init__(self):
        #citesc un fisier xml cu datele punctelor de interes din camera
        self.filename = "";
        rospy.Subscriber("experiment/photos", std_msgs.msg.String, self.photos_command_subscriber_callback);
        self.state = "STOP";
        ## face_detec
        self.cvBridge = cv_bridge.CvBridge();
        rospack = rospkg.RosPack();
        self.path_prefix = rospack.get_path('experiment_package') + "/logs/";
        self.room = "";

    def photos_command_subscriber_callback(self, info):
        print "[PhotoWriter] received info {}".format(info);
        command = json.loads(info.data);
        if(command['type'] == "START"):
            self.room = command['room'];
            self.state = "START";
        else:
            self.state = "STOP";
            self.room = "";

    def write_photo(self):
        print "[PhotoWriter] WRITING PHOTO state {}".format(self.state);
        if(self.state == "START"):
            try:
                reply = rospy.wait_for_message(
                '/xtion/rgb/image_rect_color',
                sensor_msgs.msg.Image, 1);
                frame = self.cvBridge.imgmsg_to_cv2(reply, 'bgr8');
                cv2.imwrite(self.path_prefix + self.room + rospy.get_time(), frame);
            except rospy.exceptions.ROSException:
                rospy.loginfo("[ERROR][PhotoWriter] No Info from /xtion/rgb/image_rect_color");
                return -1;

    

if __name__ == '__main__':
    rospy.init_node('experiment_photo_writer', anonymous=True);
    photoWriter = PhotoWriter();
    rate = rospy.Rate(1);
    while not rospy.is_shutdown():
        photoWriter.write_photo();
        rate.sleep();
