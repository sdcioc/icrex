#! /usr/bin/python
import json

import rospy
import std_msgs.msg
import visualization_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
class MapDraw:
    #constructor
    def __init__(self, filename):
        #citesc un fisier xml cu datele punctelor de interes din camera
        self.filename = filename;
        self.points = [];
        #self.fd = open(self.filename, 'r');
        self.marker_pub = rospy.Publisher('visualization_marker', 
                                        visualization_msgs.msg.Marker, latch=True, queue_size=10)
        rospy.sleep(1);
        self.rate = rospy.Rate(10);
        self.my_points = [json.loads(line.rstrip('\n') for line in open(self.filename, 'r'))]
        self.my_points.sort(key=lambda point: float(point['time']));
        # de modificat gresit
        for i in range((len(self.my_points)/2)):
            to_publish = visualization_msgs.msg.Marker();
            to_publish.header.frame_id = "/map";
            to_publish.header.stamp = rospy.get_time();
            to_publish.ns = "experiment";
            to_publish.action = visualization_msgs.msg.ADD;
            to_publish.type = visualization_msgs.msg.ARROW;
            to_publish.id = i;
            to_publish.points.append(geometry_msgs.msg.Point(self.my_points[2*i].position.x, self.my_points[2*i].position.y, self.my_points[2*i].position.z));
            to_publish.points.append(geometry_msgs.msg.Point(self.my_points[2*i+1].position.x, self.my_points[2*i+1].position.y, self.my_points[2*i+1].position.z));

            to_publish.scale.x = 0.1;
            to_publish.scale.y = 0.2;
            to_publish.scale.z = 0.2;
            #
        
            to_publish.color.a = 1.0;
            to_publish.color.r = 0.5;
            to_publish.color.g = 0.1;
            to_publish.color.b = 0.1;

            self.marker_pub.publish(to_publish);
            self.rate.sleep();
            """
        to_publish = {};
        to_publish['pose'] = {};
        to_publish['pose']['position'] = {};
        to_publish['pose']['position']['x'] = 1.0;
        to_publish['pose']['position']['y'] = 1.0;
        to_publish['pose']['position']['z'] = 1.0;
        to_publish['pose']['orientation'] = {};
        to_publish['pose']['orientation']['x'] = 1.0;
        to_publish['pose']['orientation']['y'] = 1.0;
        to_publish['pose']['orientation']['z'] = 1.0;
        to_publish['pose']['orientation']['w'] = 1.0;
        to_publish['time'] = rospy.get_time();
        marker = visualization_msgs.msg.Marker(
                type=visualization_msgs.msg.Marker.ARROW,
                id=0,
                pose=geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0.5, 0.5, 1.45), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
                scale=geometry_msgs.msg.Vector3(0.06, 0.06, 0.06),
                header=std_msgs.msg.Header(frame_id='/map'),
                color=std_msgs.msg.ColorRGBA(0.0, 1.0, 0.0, 0.8)
                );
        a = visualization_msgs.msg.Marker()
        a.header.frame_id = "/map"
        a.header.stamp = rospy.get_time();
        a.ns = "bezier"
        a.action = visualization_msgs.msg.ADD
        a.type = visualization_msgs.msg.ARROW
        a.id = 0
        a.points.append(geometry_msgs.msg.Point(0.5, 0.5, 1.45));
        a.points.append(geometry_msgs.msg.Point(1, 0.5, 1.45));

        a.scale.x = 0.1
        a.scale.y = 0.2
        a.scale.z = 0.2
        #
    
        a.color.a = 1.0
        a.color.r = 0.5
        a.color.g = 0.1
        a.color.b = 0.1

        self.marker_pub.publish(a);
        """

if __name__ == '__main__':
    rospy.init_node('experiment_map_draw', anonymous=True);  
    filename = rospy.get_param('~filename', '/home/pal/default_path.json')
    print "[INFO][EVENTS_WRITER] filename : {}".format(filename);
    #test_Writer_classes();
    pathWriter = MapDraw(filename);
    try:
        rospy.spin();
    except KeyboardInterrupt:
        pass;
