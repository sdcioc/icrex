#! /usr/bin/python

# Author: Ciocirlan Stefan-Dan 19/09/2018

import json
import copy

import rospy

def convert_POIName_RosparamName(poi_name):
	prefix = '/mmap/poi/submap_0/';
	if not poi_name.startswith(prefix):
		poi_name = prefix + poi_name;
	return poi_name;

class POILoader:
	#constructor
	def __init__(self, filename):
		#citesc un fisier xml cu datele punctelor de interes din camera
		self.filename = filename;
		with open(self.filename) as fd:
			self.points = json.loads(fd.read());

	def set_POIs(self):
		for point in self.points:
			path = convert_POIName_RosparamName(point['poi_name']);
			poi_data = ['submap_0', point['poi_name'], point['x'], point['y'], point['w']];
			rospy.set_param(path, poi_data);

class POISaver:
	#constructor
	def __init__(self, filename):
		#citesc un fisier xml cu datele punctelor de interes din camera
		self.filename = filename;
		self.points = [];

	def save_POIs(self):
		params = rospy.get_param_names();
		prefix = '/mmap/poi/submap_0/';
		params = filter(lambda x: x.startswith(prefix), params);
		poi_names = [p[len(prefix):] for p in params];
		for poi_name in poi_names:
			path = convert_POIName_RosparamName(poi_name)
			rosparam_data = rospy.get_param(path)
			new_data = {};
			new_data['poi_name'] = poi_name;
			new_data['x'] = rosparam_data[2];
			new_data['y'] = rosparam_data[3];
			new_data['w'] = rosparam_data[4];
			self.points.append(copy.deepcopy(new_data));
		data_to_write = json.dumps(self.points);
		with open(self.filename, 'w') as fd:
			fd.write(data_to_write);

	def save_rooms(self):
		params = rospy.get_param_names();
		prefix = '/mmap/poi/submap_0/';
		params = filter(lambda x: x.startswith(prefix), params);
		poi_names = [p[len(prefix):] for p in params];
		for poi_name in poi_names:
			if ("parent" in poi_name):
				rospy.loginfo("parent_node {}".format(poi_name))
			else:
				rospy.loginfo("room_node {}".format(poi_name))
				new_data = {};
				new_data['poi_name'] = poi_name;
				new_data['parent_poi_name'] = "parent_" + poi_name;
				new_data['visited'] = False;
				self.points.append(copy.deepcopy(new_data));
		data_to_write = json.dumps(self.points);
		with open(self.filename, 'w') as fd:
			fd.write(data_to_write);

def test_POI_classes():
    poiSaver = POISaver("/home/pal/poiinfo_saver.json")
    poiSaver.save_POIs();
    poiLoader= POILoader("/home/pal/poiinfo_loader.json");
    poiLoader.set_POIs();

if __name__ == '__main__':
    rospy.init_node('experiment_poi_manager', anonymous=True);
    operation_type = rospy.get_param('~operation_type', 'load')    
    filename = rospy.get_param('~filename', '/home/pal/default_pois.json')
    print "[INFO][POI_MANAGER] operation type : {} ; filename : {}".format(operation_type, filename);
    if (operation_type == "load"):
        poiLoader= POILoader(filename);
        poiLoader.set_POIs();
    elif (operation_type == "save"):
        poiSaver = POISaver(filename);
        poiSaver.save_POIs();
    elif (operation_type == "rooms"):
        poiSaver = POISaver(filename);
        poiSaver.save_rooms();
    else:
        print "[ERROR][POI_MANAGER] wrong operation type please use load or save";
