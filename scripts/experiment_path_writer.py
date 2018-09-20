import json

import rospy
import std_msgs.msg

class PathWriter:
    #constructor
    def __init__(self, filename):
        #citesc un fisier xml cu datele punctelor de interes din camera
        self.filename = filename;
        self.points = [];
        self.fd = open(self.filename, 'a');
        rospy.Subscriber("experiment/path", std_msgs.msg.String, self.path_subscriber_callback);

    def path_subscriber_callback(self, info):
        print "[PathWriter] received info {}".format(info);
        self.fd.write(info.data + '\n');
    
    def close(self):
        self.fd.close();

if __name__ == '__main__':
    rospy.init_node('experiment_path_writer', anonymous=True);  
    filename = rospy.get_param('~filename', '/home/pal/default_path.json')
    print "[INFO][EVENTS_WRITER] filename : {}".format(filename);
    #test_Writer_classes();
    pathWriter = PathWriter(filename);
    try:
        rospy.spin();
    except KeyboardInterrupt:
        pathWriter.close();
