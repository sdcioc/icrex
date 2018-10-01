#! /usr/bin/python
import json
import math

import rospy

class DistanceCalculator:
    #constructor
    def __init__(self):
        self.TIME = 0;

    def calculate_distance_for(self, filename):
        my_points = [json.loads(line.rstrip('\n')) for line in open(filename, 'r')]
        my_points = [x for x in my_points if float(x['time']) > self.TIME];
        my_points.sort(key=lambda point: float(point['time']));
        
        
        last_point = None;
        total_distance = 0;
        for  my_point in my_points:
            if (last_point != None):
                my_dist = self.distance(last_point, my_point);
                if(my_dist > 10):
                    last_point = my_point;
                else:
                    total_distance = total_distance + my_dist;
            last_point = my_point;
        self.TIME = float(last_point['time']);
        return total_distance;
    
    def distance(self, point1, point2):
        x1 = point1['pose']['position']['x'];
        y1 = point1['pose']['position']['y'];
        x2 = point2['pose']['position']['x'];
        y2 = point2['pose']['position']['y'];
        return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));


if __name__ == '__main__':
    rospy.init_node('experiment_distance_node', anonymous=True);  
    print "[INFO][DISTANCE_NODE] STARTED ";
    distanceCalculator = DistanceCalculator();
    try:
        print "[INFO][DISTANCE_NODE] first distance {}".format(distanceCalculator.calculate_distance_for('/home/ciocirlan/11_path.json'));
        print "[INFO][DISTANCE_NODE] second distance {}".format(distanceCalculator.calculate_distance_for('/home/ciocirlan/1_path.json'));
        rospy.spin();
    except KeyboardInterrupt:
        pass;
