#! /usr/bin/python
import rospy
import std_msgs.msg
import sensor_msgs.msg

## face detection libraries
import math
import json


class LaserLogicManager:
	#constructor
    def __init__(self):
        self.rate = rospy.Rate(10);
        self.distance_pub = rospy.Publisher(
                    '/experiment/door_distance',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        rospy.sleep(3);
        rospy.Subscriber("/scan", sensor_msgs.msg.LaserScan, self.laser_subscriber_callback);
        rospy.sleep(3);


    def laser_subscriber_callback(self, reply):
        #rospy.loginfo("[laser_subscriber_callback] message format for scan {}".format(reply));
        ranges = reply.ranges;
        new_ranges = [x for x in ranges[280:380] if not math.isnan(x)];
        average_dist = sum(new_ranges)/len(new_ranges);
        to_publish = {};
        to_publish['distance'] = average_dist;
        to_publish['time'] = rospy.get_time();
        self.distance_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        if(average_dist > 1.5):
            rospy.loginfo("[check_door_open] open");
        else:
            rospy.loginfo("[check_door_open] closed");

    def check_door_open(self):
        try:
            reply = rospy.wait_for_message(
            '/scan',
            sensor_msgs.msg.LaserScan, 3);
        except rospy.exceptions.ROSException:
            rospy.loginfo("[ERROR][check_door_open] No Info from /scan");
            return -1;
        ranges = reply.ranges;
        #rospy.loginfo("[check_door_open] message format for scan {}".format(reply));
        #rightIndexes = [];
        #for i in range(len(ranges)):
        #    if(ranges[i] < 1.0):
        #        rightIndexes.append(i);
        #mid_index = len(reply.ranges);
        #new_ranges = [x for x in ranges[mid_index-5:mid_index+5] if not math.isnan(x)];
        new_ranges = [x for x in ranges[280:380] if not math.isnan(x)];
        average_dist = sum(new_ranges)/len(new_ranges);
        rospy.loginfo("[check_door_open] new_ranges {}, \n distance {}:".format(new_ranges, average_dist));
        if(average_dist > 1.5):
            return 1;
        else:
            return 0;

if __name__ == '__main__':
    rospy.init_node('experiment_laser_node', anonymous=True);
    try:
        rospy.loginfo("[EXPERIMENT_LASER_NODE] STARTED");
        my_logic_manager = LaserLogicManager();
        #while(True):
        #    n = my_logic_manager.check_door_open();
        #    rospy.sleep(3);
        #    if(n == -1):
        #        break;
        my_logic_manager = LaserLogicManager();
        rospy.spin();
    except KeyboardInterrupt:
        pass;