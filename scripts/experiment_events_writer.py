import json

import rospy
import std_msgs.msg

class EventsWriter:
    #constructor
    def __init__(self, filename):
        #citesc un fisier xml cu datele punctelor de interes din camera
        self.filename = filename;
        self.points = [];
        self.fd = open(self.filename, 'a');
        rospy.Subscriber("experiment/events", std_msgs.msg.String, self.events_subscriber_callback);

    def events_subscriber_callback(self, info):
        print "[EventsWriter] received info {}".format(info);
        self.fd.write(info.data + '\n');

    def close(self):
        self.fd.close();

def test_Writer_classes():
    events_pub = rospy.Publisher(
                '/experiment/events',
                    std_msgs.msg.String,
                    latch=True);
    to_publish = {};
    to_publish['event'] = {};
    to_publish['event']['name'] = "TESTING THE WRITER1";
    to_publish['time'] = rospy.get_time();
    events_pub.publish(json.dumps(to_publish));

if __name__ == '__main__':
    rospy.init_node('experiment_events_writer', anonymous=True);  
    filename = rospy.get_param('~filename', '/home/pal/default_events.json')
    print "[INFO][EVENTS_WRITER] filename : {}".format(filename);
    #test_Writer_classes();
    eventsWriter = EventsWriter(filename);
    try:
        rospy.spin();
    except KeyboardInterrupt:
        eventsWriter.close();
