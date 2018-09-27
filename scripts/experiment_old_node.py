#! /usr/bin/python

# Author: Ciocirlan Stefan-Dan 19/09/2018


import math
import sys
import json
import copy

import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import pal_web_msgs.msg

## face detection libraries
import cv2
import dlib
import cv_bridge

#sound
import subprocess
import random
import rospkg

def convert_POIPosition_MapPosition(position):
	#tipul de mesaj pentru map
	pos = geometry_msgs.msg.PoseStamped();
	#harta pe care are loc pozitionarea si directia
	pos.header.frame_id = 'map';
	pos.pose.position.x = position[0];
	pos.pose.position.y = position[1];
	pos.pose.orientation.z = math.sin(position[2] / 2.0);
	pos.pose.orientation.w = math.cos(position[2] / 2.0);
	return pos;

#intrare geometry_msgs.msg.PoseStamped (vector3 position, vector3 orientation)
def convert_MapPosition_POIPosition(position):
        x = position.position.x;
        y = position.position.y;
        w = 2 * math.acos(position.orientation.w);
        return (x, y, w);

def convert_POIName_RosparamName(poi_name):
	prefix = '/mmap/poi/submap_0/';
	if not poi_name.startswith(prefix):
		poi_name = prefix + poi_name;
	return poi_name;

class POILocationManager:
	#constructor
	def __init__(self):
		self.prefix = '/mmap/poi/submap_0/';

	#returneaza pozitia pe harta a punctului de interes
	def get_position(self, poi_name):
		print "[INFO] getting position for poi {}".format(poi_name) 
		poi_name = convert_POIName_RosparamName(poi_name)
		try:
			poi = rospy.get_param(poi_name)
			if not poi:
				return None
			if len(poi[2:]) != 3:
				return None
			position = convert_POIPosition_MapPosition(poi[2:])
			return position
		except KeyError:
			return None


#Clasa ce ofera informatiile despre puncte de interes din camere
class POIInfoManager:
    #constructor
    def __init__(self, filename):
        #citesc un fisier xml cu datele punctelor de interes din camera
        self.filename = filename;
        with open(self.filename) as fd:
            self.points = json.loads(fd.read());
            self.current = 0;
            for point in self.points:
                point['tried'] = False;

    # returneaza numele punctului de interes parinte
    def get_parent_poi_name(self):
        return self.points[self.current]['parent_poi_name'];

    # returneanza numele punctului de interes curent
    def get_poi_name(self):
        return self.points[self.current]['poi_name'];

    # returneaza numarul de oameni aflati in sala
    def get_number_of_perople(self):
        return self.points[self.current]['people_number'];

    # trece la urmatoare camera nevizitata
    def next_poi(self):
        self.points[self.current]['tried'] = True;
        last_pos = self.current;
        self.current = (self.current + 1) % len(self.points);
        while(last_pos != self.current):
            if(self.points[self.current]['visited'] == False and self.points[self.current]['tried'] == False):
                return True;
            else:
                self.current = (self.current + 1) % len(self.points);
        return False;
	
    # a fost vizitat nodul curent
    def set_visited(self):
        self.points[self.current]['visited'] = True;

    def set_poi(self, poi_name):
        for point in self.points:
            if (poi_name == point['poi_name']):
                self.current = self.points.index(point);
                return True;
        return False;

    # scrie ce am vizitat azi
    def write_file(self):
        for point in self.points:
            del point['tried'];
        data_to_write = json.dumps(self.points);
        with open(self.filename, 'w') as fd:
            fd.write(data_to_write);

class SoundManager:
    def __init__(self, configfile):
        self.filename = configfile;
        with open(self.filename) as fd:
            self.files = json.loads(fd.read());
        self.utility = "aplay ";
        self.general_volume_param = "/pal/general_volume";
        self.playback_volume_param = "/pal/playback_volume";
        rospy.set_param(self.general_volume_param, 60);
        rospy.set_param(self.playback_volume_param, 60);
        rospack = rospkg.RosPack();
        self.prefix = rospack.get_path('experiment_package') + "/config/sounds/";
    
    def play_first(self, guessed_type):
        command = self.utility + self.prefix;
        language = rospy.get_param("/user_lang")[0:2];
        command = command + language + "_first_";
        if(guessed_type == 1):
            command = command + "promotion.wav";
        else:
            command = command + "prevention.wav";
        subprocess.check_output(command.split());

    def play_second(self, guessed_type):
        command = self.utility + self.prefix;
        language = rospy.get_param("/user_lang")[0:2];
        command = command + language + "_second_";
        if(guessed_type == 1):
            command = command + "promotion.wav";
        else:
            command = command + "prevention.wav";
        subprocess.check_output(command.split());

    def play_final(self, guessed_type):
        command = self.utility + self.prefix;
        language = rospy.get_param("/user_lang")[0:2];
        command = command + language + "_final_";
        if(guessed_type == 1):
            command = command + "promotion.wav";
        else:
            command = command + "prevention.wav";
        subprocess.check_output(command.split());

    def play_exit(self, guessed_type):
        command = self.utility + self.prefix;
        language = rospy.get_param("/user_lang")[0:2];
        command = command + language + "_exit_";
        if(guessed_type == 1):
            command = command + "promotion.wav";
        else:
            command = command + "prevention.wav";
        subprocess.check_output(command.split());

    def play_initial(self):
        command = self.utility + self.prefix;
        language = rospy.get_param("/user_lang")[0:2];
        command = command + language + "_initial.wav";
        subprocess.check_output(command.split());

    def play_much(self):
        command = self.utility + self.prefix;
        language = rospy.get_param("/user_lang")[0:2];
        command = command + language + "_much.wav";
        subprocess.check_output(command.split());
    

class ExperimentLogicManager:
	#constructor
    def __init__(self, infofilename, soundfilename):
        self.poi_info_manager = POIInfoManager(infofilename);
        self.poi_location_manager = POILocationManager();
        self.sound_manager = SoundManager(soundfilename);
        self.state = "INITIAL";
        self.guessed_type = 0;
        self.DISTANCE_ERROR = 0.6;
        self.sonar_param = "/speed_limit/limitess/base_sonars/obstacle_max/dist";
        self.laser_param = "/speed_limit/limitess/base_laser/obstacle_max/dist";
        self.obstacle_normal_dist = 0.5;
        self.obstacle_door_dist = 0.1;
        self.people_number = 0;
        self.rate = rospy.Rate(10);
        self.events_pub = rospy.Publisher(
                    '/experiment/events',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        self.path_pub = rospy.Publisher(
                    '/experiment/path',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        self.command_pub = rospy.Publisher(
                    '/experiment/cmd',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        self.move_pub = rospy.Publisher(
                    '/move_base_simple/goal',
                        geometry_msgs.msg.PoseStamped,
                        latch=True, queue_size=5);
        self.web_pub = rospy.Publisher(
                    '/web/go_to',
                        pal_web_msgs.msg.WebGoTo,
                        latch=True, queue_size=5);
        rospy.sleep(3);
        rospy.Subscriber("experiment/cmd", std_msgs.msg.String, self.command_subscriber_callback);
        rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, self.position_subscriber_callback);
        rospy.sleep(3);
        ## face_detec
        self.cvBridge = cv_bridge.CvBridge();
        self.faceDectector = dlib.get_frontal_face_detector();

    def command_subscriber_callback(self, ros_string_command):
        rospy.loginfo("[COMMAND_CALLBACK] received command {}".format(ros_string_command));
        current_command = json.loads(ros_string_command.data);
        #execute de command
        if ( (self.state is "STOP") and (not 'manual' in current_command)):
            rospy.loginfo("[COMMAND_CALLBACK] going from state {} to WAITING_COMMAND last command {}".format(self.state, current_command['type']));
            self.state = "WAITING_COMMAND"
        elif (current_command['type'] == "FORCE_POI"):
            self.force_poi_command(current_command);
        elif ((self.state is "STOP") and ('manual' in current_command)):
             rospy.loginfo("[COMMAND_CALLBACK] going from state {} please wait to stop autonomous commands yout command {}".format(self.state, current_command['type']));   
        else:
            if (current_command['type'] == "START_NEXT_POI"):
                self.start_next_poi_command(current_command);
            elif (current_command['type'] == "NEXT_POI"):
                self.next_poi_command(current_command);
            elif (current_command['type'] == "FINISH_EXPERIMENT"):
                self.finish_experiment_command(current_command);
            elif (current_command['type'] == "GOTO_PARENT_POI"):
                self.goto_parent_poi_command(current_command);
            elif (current_command['type'] == "VERIFY_DISTANCE_PARENT_POI"):
                self.verify_distance_parent_poi_command(current_command);
            elif (current_command['type'] == "VERIFY_DOOR"):
                self.verify_door_command(current_command);
            elif (current_command['type'] == "GOTO_POI"):
                self.goto_poi_command(current_command);
            elif (current_command['type'] == "VERIFY_DISTANCE_POI"):
                self.verify_distance_poi_command(current_command);
            elif (current_command['type'] == "CHECK_PEOPLE"):
                self.check_people_command(current_command);
            elif (current_command['type'] == "GOTO_BACK_PARENT_POI"):
                self.goto_back_parent_poi_command(current_command);
            elif (current_command['type'] == "VERIFY_DISTANCE_BACK_PARENT_POI"):
                self.verify_distance_back_parent_poi_command(current_command);
            elif (current_command['type'] == "DO_EXPERIMENT"):
                self.do_experiment_command(current_command);
            elif (current_command['type'] == "STOP"):
                self.stop_command(current_command);
            elif (current_command['type'] == "START_EXPERIMENT"):
                self.start_next_poi_command(current_command);
            else:
                rospy.loginfo("[COMMAND_CALLBACK] state {} Wrong Command type command {}".format(self.state, current_command['type']));
 
    def position_subscriber_callback(self, reply):
        print "[POSITION] {}".format(reply.pose.pose);
        self.current_position = reply.pose.pose;
        to_publish = {};
        to_publish['pose'] = {};
        to_publish['pose']['position'] = {};
        to_publish['pose']['position']['x'] = self.current_position.position.x;
        to_publish['pose']['position']['y'] = self.current_position.position.y;
        to_publish['pose']['position']['z'] = self.current_position.position.z;
        to_publish['pose']['orientation'] = {};
        to_publish['pose']['orientation']['x'] = self.current_position.orientation.x;
        to_publish['pose']['orientation']['y'] = self.current_position.orientation.y;
        to_publish['pose']['orientation']['z'] = self.current_position.orientation.z;
        to_publish['pose']['orientation']['w'] = self.current_position.orientation.w;
        to_publish['time'] = rospy.get_time();
        self.path_pub.publish(json.dumps(to_publish));
        self.rate.sleep();

    def force_poi_command(self, command):
        rospy.loginfo("[FORCE_POI] going from state {} to MANUAL_COMPLETE".format(self.state));
        self.state = "MANUAL_COMPLETE"
        self.move_pub.publish(self.poi_location_manager.get_position(command['poi_name']));
        self.rate.sleep();

    def start_next_poi_command(self, command):
        rospy.loginfo("[START_NEXT_POI] going from state {} to IDLE".format(self.state));
        self.state = "IDLE"
        if 'manual' in command:
            rospy.loginfo("[START_NEXT_POI] manual command poi {}".format(command['poi_name']));
            self.poi_info_manager.set_poi(command['poi_name']);
        else:
            rospy.loginfo("[START_NEXT_POI] autonomus command");
        self.parent_poi_name = self.poi_info_manager.get_parent_poi_name();
        self.poi_name = self.poi_info_manager.get_poi_name();
        self.parent_poi_position = self.poi_location_manager.get_position(self.parent_poi_name);
        self.poi_position = self.poi_location_manager.get_position(self.poi_name);
        rospy.loginfo("[START_NEXT_POI] Room: {}".format(self.poi_name));
        rospy.loginfo("[START_NEXT_POI] Goint to {}".format(self.parent_poi_name));
        to_publish = {};
        to_publish['room'] = self.poi_name;
        to_publish['event'] = {};
        to_publish['event']['name'] = "SELECTING NEXT POINT {} WITH PARENT {}".format(self.poi_name, self.parent_poi_name);
        to_publish['event']['type'] = "GOTO_ROOM";
        to_publish['time'] = rospy.get_time();
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        next_command = {};
        next_command['type'] = "GOTO_PARENT_POI";
        rospy.sleep(1);
        rospy.loginfo("[START_NEXT_POI] publish next command");
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    def next_poi_command(self, command):
        rospy.loginfo("[NEXT_POI] going from state {} to GETTING_NEXT_POI".format(self.state));
        self.state = "GETTING_NEXT_POI"
        next_command = {}
        if(self.poi_info_manager.next_poi() == False):
            next_command['type'] = "FINISH_EXPERIMENT"
        else:
            next_command['type'] = "START_NEXT_POI"
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    def finish_experiment_command(self, command):
        rospy.loginfo("[FINISH_EXPERIMENT] BIG EXPERIMENT FINISHED ALL DOORS VISITED OR TRIED");
        rospy.loginfo("[FINISH_EXPERIMENT] going from state {} to EXPERIMENT_FINISHED".format(self.state));
        self.state = "EXPERIMENT_FINISHED";
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "FINSIH EXPERIMENT";
        to_publish['event']['type'] = "EXPERIMENT_FINISHED";
        to_publish['time'] = rospy.get_time();
        self.poi_info_manager.write_file();
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();

    def goto_parent_poi_command(self, command):
        rospy.loginfo("[GOTO_PARENT_POI] going from state {} to GOING_TO_PARENT_POI".format(self.state));
        self.state = "GOING_TO_PARENT_POI"
        self.move_pub.publish(self.parent_poi_position);
        next_command = {};
        next_command['type'] = "VERIFY_DISTANCE_PARENT_POI";
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
        
    def verify_distance_parent_poi_command(self, command):
        rospy.loginfo("[VERIFY_DISTANCE_PARENT_POI] going from state {} to GOING_TO_PARENT_POI".format(self.state));
        self.state = "GOING_TO_PARENT_POI"
        current_distance = self.get_distance(self.current_position, self.parent_poi_position);
        next_command = {}
        if ( current_distance > self.DISTANCE_ERROR):
            next_command['type'] = "VERIFY_DISTANCE_PARENT_POI";
            rospy.sleep(2);
        else:
            rospy.loginfo("[VERIFY_DISTANCE_PARENT_POI] going from state {} to ARRIVED_PARENT_POI".format(self.state));
            self.state = "ARRIVED_PARENT_POI"
            to_publish = {};
            to_publish['event'] = {};
            to_publish['event']['name'] = "ARRIVED PARENT POINT {}".format(self.parent_poi_name);
            to_publish['time'] = rospy.get_time();
            to_publish['event']['type'] = "ARRIVED_ROOM_DOOR";
            to_publish['room'] = self.poi_name;
            self.events_pub.publish(json.dumps(to_publish));
            self.rate.sleep();
            next_command['type'] = "VERIFY_DOOR";
            rospy.sleep(3);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();


    def manual_verify_door_command(self, command):
        rospy.loginfo("[VERIFY_DOOR] going from state {} to VERIFING_DOOR".format(self.state));
        self.state = "VERIFING_DOOR"
        next_command = {}
        to_publish = {};
        to_publish['event'] = {};
        if(command['door'] == 1):
            rospy.loginfo("[VERIFY_DOOR] Door opened")
            rospy.set_param(self.sonar_param, self.obstacle_door_dist);
            rospy.set_param(self.laser_param, self.obstacle_door_dist);
            next_command['type'] = "GOTO_POI";
            to_publish['event']['name'] = "DOOR OPEN POINT {} WITH PARENT {}".format(self.poi_name, self.parent_poi_name);
            to_publish['event']['type'] = "DOOR_OPEN";
        else:
            rospy.loginfo("[VERIFY_DOOR] Door closed")
            next_command['type'] = "NEXT_POI";
            to_publish['event']['name'] = "DOOR CLOSED POINT {} WITH PARENT {}".format(self.poi_name, self.parent_poi_name);
            to_publish['event']['type'] = "DOOR_CLOSED";
        to_publish['time'] = rospy.get_time();
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    def verify_door_command(self, command):
        rospy.loginfo("[VERIFY_DOOR] going from state {} to VERIFING_DOOR".format(self.state));
        self.state = "VERIFING_DOOR"
        next_command = {}
        to_publish = {};
        to_publish['event'] = {};
        if(self.check_door_open() == 1):
            rospy.loginfo("[VERIFY_DOOR] Door opened")
            rospy.set_param(self.sonar_param, self.obstacle_door_dist);
            rospy.set_param(self.laser_param, self.obstacle_door_dist);
            next_command['type'] = "GOTO_POI";
            to_publish['event']['name'] = "DOOR OPEN POINT {} WITH PARENT {}".format(self.poi_name, self.parent_poi_name);
        else:
            rospy.loginfo("[VERIFY_DOOR] Door closed")
            next_command['type'] = "NEXT_POI";
            to_publish['event']['name'] = "DOOR CLOSED POINT {} WITH PARENT {}".format(self.poi_name, self.parent_poi_name);
        to_publish['time'] = rospy.get_time();
        #self.events_pub.publish(json.dumps(to_publish));
        #self.rate.sleep();
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();


    def goto_poi_command(self, command):
        rospy.loginfo("[GOTO_POI] going from state {} to GOING_TO_POI".format(self.state));
        self.state = "GOING_TO_POI"
        self.move_pub.publish(self.poi_position);
        next_command = {};
        next_command['type'] = "VERIFY_DISTANCE_POI";
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
        
    def verify_distance_poi_command(self, command):
        rospy.loginfo("[VERIFY_DISTANCE_POI] going from state {} to GOING_TO_POI".format(self.state));
        current_distance = self.get_distance(self.current_position, self.poi_position);
        next_command = {}
        if ( current_distance > self.DISTANCE_ERROR):
            next_command['type'] = "VERIFY_DISTANCE_POI";
            rospy.sleep(2);
            self.command_pub.publish(json.dumps(next_command));
            self.rate.sleep();
        else:
            rospy.loginfo("[VERIFY_DISTANCE_POI] going from state {} to ARRIVED_POI".format(self.state));
            self.state = "ARRIVED_POI"
            to_publish = {};
            to_publish['event'] = {};
            to_publish['event']['name'] = "ARRIVED POINT {}".format(self.poi_name);
            to_publish['time'] = rospy.get_time();
            to_publish['event']['type'] = "ARRIVED_ROOM";
            to_publish['room'] = self.poi_name;
            self.events_pub.publish(json.dumps(to_publish));
            self.rate.sleep();
            rospy.set_param(self.sonar_param, self.obstacle_normal_dist);
            rospy.set_param(self.laser_param, self.obstacle_normal_dist);
            next_command['type'] = "CHECK_PEOPLE";
            #TODO: START MAKING PHOTOS
        #self.command_pub.publish(json.dumps(next_command));
        #self.rate.sleep();
    

    def manual_check_people_command(self, command):
        rospy.loginfo("[MANUAL_CHECK_PEOPLE] going from state {} to CHECKING_PEOPLE".format(self.state));
        next_command = {}
        self.state = "CHECKING_PEOPLE"
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "START CHECKING PEOPLE POINT {}".format(self.parent_poi_name);
        to_publish['time'] = rospy.get_time();
        to_publish['event']['type'] = "CHECK_PEOPLE";
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        self.people_number = command['people_number'];
        rospy.sleep(1);
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "FOUND PEOPLE POINT {}".format(self.parent_poi_name);
        to_publish['time'] = rospy.get_time();
        to_publish['event']['type'] = "FOUND_PEOPLE";
        to_publish['event']['number'] = self.people_number;
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        if( (self.people_number < 3) and (self.people_number > 0)):
            next_command['type'] = "DO_EXPERIMENT";
        else:
            next_command['type'] = "GOTO_BACK_PARENT_POI";
            rospy.set_param(self.sonar_param, self.obstacle_door_dist);
            rospy.set_param(self.laser_param, self.obstacle_door_dist);
            self.command_pub.publish(json.dumps(next_command));
            self.rate.sleep();
        #self.command_pub.publish(json.dumps(next_command));
        #self.rate.sleep();

    def check_people_command(self, command):
        rospy.loginfo("[CHECK_PEOPLE] going from state {} to CHECKING_PEOPLE".format(self.state));
        next_command = {}
        self.state = "CHECKING_PEOPLE"
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "START CHECKING PEOPLE POINT {}".format(self.parent_poi_name);
        to_publish['time'] = rospy.get_time();
        to_publish['event']['type'] = "CHECK_PEOPLE";
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        self.people_number = self.check_number_people();
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "FOUND PEOPLE POINT {}".format(self.parent_poi_name);
        to_publish['time'] = rospy.get_time();
        to_publish['event']['type'] = "FOUND_PEOPLE";
        to_publish['event']['number'] = self.people_number;
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        if( (self.people_number < 3) and (self.people_number > 0)):
            next_command['type'] = "DO_EXPERIMENT";
        else:
            next_command['type'] = "GOTO_BACK_PARENT_POI";
            rospy.set_param(self.sonar_param, self.obstacle_door_dist);
            rospy.set_param(self.laser_param, self.obstacle_door_dist);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    def goto_back_parent_poi_command(self, command):
        rospy.loginfo("[GOTO_BACK_PARENT_POI] going from state {} to GOING_TO_BACK_PARENT_POI".format(self.state));
        self.state = "GOING_TO_BACK_PARENT_POI"
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "GO bACK parent POINT {}".format(self.parent_poi_name);
        to_publish['time'] = rospy.get_time();
        to_publish['event']['type'] = "LEAVING_ROOM";
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        self.sound_manager.play_exit(self.guessed_type);
        self.move_pub.publish(self.parent_poi_position);
        self.rate.sleep();
        #TODO: STOP_MAKING_PHOTOS
        next_command = {};
        next_command['type'] = "VERIFY_DISTANCE_BACK_PARENT_POI";
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
        
    def verify_distance_back_parent_poi_command(self, command):
        rospy.loginfo("[VERIFY_DISTANCE_BACK_PARENT_POI] going from state {} to GOING_TO_BACK_PARENT_POI".format(self.state));
        current_distance = self.get_distance(self.current_position, self.parent_poi_position);
        next_command = {}
        if ( current_distance > self.DISTANCE_ERROR):
            next_command['type'] = "VERIFY_DISTANCE_BACK_PARENT_POI";
            rospy.sleep(2);
        else:
            next_command['type'] = "NEXT_POI";
            rospy.set_param(self.sonar_param, self.obstacle_normal_dist);
            rospy.set_param(self.laser_param, self.obstacle_normal_dist);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    def do_experiment_old_command(self, command):
        rospy.loginfo("[DO_EXPERIMENT] going from state {} to DOING_EXPERIMENT".format(self.state));
        self.state = "DOING_EXPERIMENT"
        rospy.sleep(10);
        next_command = {};
        rospy.set_param(self.sonar_param, self.obstacle_door_dist);
        rospy.set_param(self.laser_param, self.obstacle_door_dist);
        next_command['type'] = "GOTO_BACK_PARENT_POI";
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "EXPERIMENT DONE POINT {}".format(self.parent_poi_name);
        to_publish['time'] = rospy.get_time();
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
    
    def do_manual_speak_to_many(self, command):
        rospy.loginfo("[MANUAL SPEAK] going from state {} to MUCH_PEOPLE".format(self.state));
        self.sound_manager.play_much();

    def do_manual_first_experiment_command(self, command):
        rospy.loginfo("[DO_EXPERIMENT_MANUAL_FIRST] going from state {} to DOING_EXPERIMENT".format(self.state));
        self.state = "DOING_EXPERIMENT"
        self.guessed_type = random.randint(0,1);
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "EXPERIMENT STARTED {}".format(self.poi_name);
        to_publish['event']['guessed_type'] = self.guessed_type;
        to_publish['event']['type'] = "START_EXPERIMENT";
        to_publish['time'] = rospy.get_time();
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();


    def do_manual_first_second_experiment_command(self, command):
        rospy.loginfo("[DO_EXPERIMENT_MANUAL_FIRST_SECOND] going from state {} to DOING_EXPERIMENT".format(self.state));
        self.state = "DOING_EXPERIMENT"
        self.guessed_type = 1 - self.guessed_type;
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "EXPERIMENT STARTED {}".format(self.poi_name);
        to_publish['event']['guessed_type'] = self.guessed_type;
        to_publish['event']['type'] = "START_EXPERIMENT";
        to_publish['time'] = rospy.get_time();
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();

    def do_manual_second_experiment_command(self, command):
        #TODO: look after first person
        rospy.sleep(1);
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "EXPERIMENT STRTED LOOK UP {}".format(self.poi_name);
        to_publish['event']['guessed_type'] = self.guessed_type;
        to_publish['event']['type'] = "SPEAKING_FIRST";
        to_publish['time'] = rospy.get_time();
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        self.sound_manager.play_first(self.guessed_type);
        self.state = "EXPERIMENT_WAITING_FIRST";
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "EXPERIMENT FINISHED INITIAL SOUND {}".format(self.poi_name);
        to_publish['event']['guessed_type'] = self.guessed_type;
        to_publish['event']['type'] = "WAITING_FIRST_RESPONSE";
        to_publish['time'] = rospy.get_time();
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        #TODO:show the questions lansare index
        web_msg = pal_web_msgs.msg.WebGoTo();
        web_msg.type = 3;
        web_msg.value = "/static/webapps/client/experiment/index.html";
        self.web_pub.publish(web_msg);
        try:
            reply = rospy.wait_for_message(
            '/experiment/web/cmd',
            std_msgs.msg.String, 30);
            to_publish = {};
            to_publish['event'] = {};
            to_publish['event']['name'] = "EXPERIMENT RESPONDED FIRST {}".format(self.poi_name);
            to_publish['event']['guessed_type'] = self.guessed_type;
            to_publish['event']['type'] = "WAITING_STOP";
            to_publish['time'] = rospy.get_time();
            to_publish['room'] = self.poi_name;
            self.events_pub.publish(json.dumps(to_publish));
            self.rate.sleep();
            rospy.loginfo("[INFO][SECOND_EXPERIMENT] WAITING COMPLETION");

            finish = rospy.wait_for_message(
            '/experiment/web/cmd',
            std_msgs.msg.String);
            to_publish = {};
            to_publish['event'] = {};
            to_publish['event']['name'] = "EXPERIMENT FINISHED FIRST {}".format(self.poi_name);
            to_publish['event']['guessed_type'] = self.guessed_type;
            to_publish['event']['type'] = "EXPERIMENT_FINISHED";
            to_publish['time'] = rospy.get_time();
            to_publish['room'] = self.poi_name;
            self.events_pub.publish(json.dumps(to_publish));
            self.rate.sleep();
            self.sound_manager.play_final(self.guessed_type);
        except rospy.exceptions.ROSException:
            rospy.loginfo("[INFO][DO_EXPERIMENT] No starrting the questions first");
            #TODO:move foward
            to_publish = {};
            to_publish['event'] = {};
            to_publish['event']['name'] = "EXPERIMENT SECOND TRY {}".format(self.poi_name);
            to_publish['event']['guessed_type'] = self.guessed_type;
            to_publish['event']['type'] = "SPEAKING_SECOND";
            to_publish['time'] = rospy.get_time();
            to_publish['room'] = self.poi_name;
            self.events_pub.publish(json.dumps(to_publish));
            self.rate.sleep();
            self.sound_manager.play_second(self.guessed_type);
            to_publish = {};
            to_publish['event'] = {};
            to_publish['event']['name'] = "EXPERIMENT SECOND {}".format(self.poi_name);
            to_publish['event']['guessed_type'] = self.guessed_type;
            to_publish['event']['type'] = "WAITING_SECOND_RESPONSE";
            to_publish['time'] = rospy.get_time();
            to_publish['room'] = self.poi_name;
            self.events_pub.publish(json.dumps(to_publish));
            self.rate.sleep();
            try:
                reply = rospy.wait_for_message(
                '/experiment/web/cmd',
                std_msgs.msg.String, 15);
                to_publish = {};
                to_publish['event'] = {};
                to_publish['event']['name'] = "EXPERIMENT RESPONDED SECOND {}".format(self.poi_name);
                to_publish['event']['guessed_type'] = self.guessed_type;
                to_publish['event']['type'] = "WAITING_STOP";
                to_publish['time'] = rospy.get_time();
                to_publish['room'] = self.poi_name;
                self.events_pub.publish(json.dumps(to_publish));
                self.rate.sleep();
                finish = rospy.wait_for_message(
                '/experiment/web/cmd',
                std_msgs.msg.String);
                to_publish = {};
                to_publish['event'] = {};
                to_publish['event']['name'] = "EXPERIMENT FINIHED SECOND {}".format(self.poi_name);
                to_publish['event']['guessed_type'] = self.guessed_type;
                to_publish['event']['type'] = "EXPERIMENT_FINISHED";
                to_publish['time'] = rospy.get_time();
                to_publish['room'] = self.poi_name;
                self.events_pub.publish(json.dumps(to_publish));
                self.rate.sleep();
                self.sound_manager.play_final(self.guessed_type);
            except rospy.exceptions.ROSException:
                rospy.loginfo("[INFO][DO_EXPERIMENT] No starrting the questions second");
                to_publish = {};
                to_publish['event'] = {};
                to_publish['event']['name'] = "EXPERIMENT NOT RESPONDED {}".format(self.poi_name);
                to_publish['event']['guessed_type'] = self.guessed_type;
                to_publish['time'] = rospy.get_time();
                self.events_pub.publish(json.dumps(to_publish));
                self.rate.sleep();

        #self.command_pub.publish(json.dumps(next_command));
        #self.rate.sleep();

    def do_experiment_command(self, command):
        rospy.loginfo("[DO_EXPERIMENT_1] going from state {} to DOING_EXPERIMENT".format(self.state));
        self.state = "DOING_EXPERIMENT"
        rospy.set_param(self.sonar_param, self.obstacle_door_dist);
        rospy.set_param(self.laser_param, self.obstacle_door_dist);
        self.guessed_type = random.randint(0,1);
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "EXPERIMENT STARTED {}".format(self.poi_name);
        to_publish['event']['guessed_type'] = self.guessed_type;
        to_publish['event']['type'] = "START_EXPERIMENT";
        to_publish['time'] = rospy.get_time();
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        #TODO: look after first person
        rospy.sleep(1);
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "EXPERIMENT STRTED LOOK UP {}".format(self.poi_name);
        to_publish['event']['guessed_type'] = self.guessed_type;
        to_publish['event']['type'] = "SPEAKING_FIRST";
        to_publish['time'] = rospy.get_time();
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        self.sound_manager.play_first(self.guessed_type);
        self.state = "EXPERIMENT_WAITING_FIRST";
        to_publish = {};
        to_publish['event'] = {};
        to_publish['event']['name'] = "EXPERIMENT FINISHED INITIAL SOUND {}".format(self.poi_name);
        to_publish['event']['guessed_type'] = self.guessed_type;
        to_publish['event']['type'] = "WAITING_FIRST_RESPONSE";
        to_publish['time'] = rospy.get_time();
        to_publish['room'] = self.poi_name;
        self.events_pub.publish(json.dumps(to_publish));
        self.rate.sleep();
        #TODO:show the questions lansare index
        web_msg = pal_web_msgs.msg.WebGoTo();
        web_msg.type = 3;
        web_msg.value = "/static/webapps/client/experiment/index.html";
        self.web_pub.publish(web_msg);
        next_command = {};
        if (self.people_number == 2):
            next_command['type'] = "DO_EXPERIMENT";
            self.people_number = self.people_number - 1;
        else:
            next_command['type'] = "GOTO_BACK_PARENT_POI";
        try:
            reply = rospy.wait_for_message(
            '/experiment/web/cmd',
            std_msgs.msg.String, 30);
            to_publish = {};
            to_publish['event'] = {};
            to_publish['event']['name'] = "EXPERIMENT RESPONDED FIRST {}".format(self.poi_name);
            to_publish['event']['guessed_type'] = self.guessed_type;
            to_publish['event']['type'] = "WAITING_STOP";
            to_publish['time'] = rospy.get_time();
            to_publish['room'] = self.poi_name;
            self.events_pub.publish(json.dumps(to_publish));
            self.rate.sleep();

            finish = rospy.wait_for_message(
            '/experiment/web/cmd',
            std_msgs.msg.String);
            self.sound_manager.play_final(self.guessed_type);
        except rospy.exceptions.ROSException:
            rospy.loginfo("[INFO][DO_EXPERIMENT] No starrting the questions first");
            #TODO:move foward
            to_publish = {};
            to_publish['event'] = {};
            to_publish['event']['name'] = "EXPERIMENT sECONF TRY? {}".format(self.poi_name);
            to_publish['event']['guessed_type'] = self.guessed_type;
            to_publish['event']['type'] = "SPEAKING_SECOND";
            to_publish['time'] = rospy.get_time();
            to_publish['room'] = self.poi_name;
            self.events_pub.publish(json.dumps(to_publish));
            self.rate.sleep();
            self.sound_manager.play_second(self.guessed_type);
            to_publish = {};
            to_publish['event'] = {};
            to_publish['event']['name'] = "EXPERIMENT sECOND {}".format(self.poi_name);
            to_publish['event']['guessed_type'] = self.guessed_type;
            to_publish['event']['type'] = "WAITING_SECOND_RESPONSE";
            to_publish['time'] = rospy.get_time();
            to_publish['room'] = self.poi_name;
            self.events_pub.publish(json.dumps(to_publish));
            self.rate.sleep();
            try:
                reply = rospy.wait_for_message(
                '/experiment/web/cmd',
                std_msgs.msg.String, 15);
                to_publish = {};
                to_publish['event'] = {};
                to_publish['event']['name'] = "EXPERIMENT RESPONDED SECOND {}".format(self.poi_name);
                to_publish['event']['guessed_type'] = self.guessed_type;
                to_publish['event']['type'] = "WAITING_STOP";
                to_publish['time'] = rospy.get_time();
                to_publish['room'] = self.poi_name;
                self.events_pub.publish(json.dumps(to_publish));
                self.rate.sleep();
                finish = rospy.wait_for_message(
                '/experiment/web/cmd',
                std_msgs.msg.String);
                self.sound_manager.play_final(self.guessed_type);
            except rospy.exceptions.ROSException:
                rospy.loginfo("[INFO][DO_EXPERIMENT] No starrting the questions second");
                to_publish = {};
                to_publish['event'] = {};
                to_publish['event']['name'] = "EXPERIMENT NOT RESPONDED {}".format(self.poi_name);
                to_publish['event']['guessed_type'] = self.guessed_type;
                to_publish['time'] = rospy.get_time();
                self.events_pub.publish(json.dumps(to_publish));
                self.rate.sleep();

        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    def stop_command(self, command):
        rospy.loginfo("[STOP] going from state {} to STOP".format(self.state));
        self.state = "STOP";
    

    def get_current_position(self):
        try:
            reply = rospy.wait_for_message(
            '/amcl_pose',
            geometry_msgs.msg.PoseWithCovarianceStamped, 3)
        except rospy.exceptions.ROSException:
            rospy.loginfo("[ERROR] No Info from amcl_pose trying slam_karto");
            try:
                reply = rospy.wait_for_message(
                '/slam_karto_pose',
                geometry_msgs.msg.PoseWithCovarianceStamped, 3)
            except rospy.exceptions.ROSException:
                rospy.loginfo("[ERROR] No Info from slam_karto");
                return None
        if(reply.header.frame_id == 'map'):
            rospy.loginfo("[ERROR] Info from another map");
            return None
        return reply.pose.pose

    def get_distance(self, point1, point2):
        x1 = point1.position.x;
        y1 = point1.position.y;
        x2 = point2.pose.position.x;
        y2 = point2.pose.position.y;
        return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    
    def check_door_open(self):
        #try:
        #    reply = rospy.wait_for_message(
        #    '/scan',
        #    sensor_msgs.msg.LaserScan, 3);
        #except rospy.exceptions.ROSException:
        #    rospy.loginfo("[ERROR][check_door_open] No Info from /scan");
        #    return -1;
        #ranges = reply.ranges;
        #mid_index = len(ranges)/2;
        #new_ranges = [x for x in ranges[mid_index-10:mid_index+1-] if not math.isnan(x)];
        ##new_ranges = [x for x in ranges[280:380] if not math.isnan(x)];
        #average_dist = sum(new_ranges)/len(new_ranges);
        #rospy.loginfo("[check_door_open] new_ranges {}, \n distance {}:".format(new_ranges, average_dist));
        #if(average_dist > 1.5):
        #    return 1;
        #else:
        #    return 0;
        return 1;

    def check_number_people_helper(self):
        try:
            reply = rospy.wait_for_message(
            '/camera/rgb/image_rect_color',
            sensor_msgs.msg.Image, 3);
        except rospy.exceptions.ROSException:
            rospy.loginfo("[ERROR][check_number_people] No Info from /camera/rgb/image_rect_color");
            return -1;
        #rospy.loginfo("[check_number_people] message format for scan {}".format(reply));
        frame = self.cvBridge.imgmsg_to_cv2(reply, 'bgr8');
        faces_detected = self.faceDectector(frame, 1);
        rospy.loginfo("[check_number_people] people detected {}".format(len(faces_detected)));
        return len(faces_detected);

    def check_number_people(self):
        returnValue = -1;
        current_guess = self.check_number_people_helper();
        if (current_guess > returnValue):
            returnValue = current_guess;
        rospy.sleep(1);
        self.sound_manager.play_initial();
        current_guess = self.check_number_people_helper();
        if (current_guess > returnValue):
            returnValue = current_guess;
        rospy.sleep(1);
        current_guess = self.check_number_people_helper();
        if (current_guess > returnValue):
            returnValue = current_guess;
        rospy.sleep(2);
        current_guess = self.check_number_people_helper();
        if (current_guess > returnValue):
            returnValue = current_guess;
        return returnValue;


def test_POI_classes():
    poiInfoManager = POIInfoManager("/home/pal/roomsinfo.json");
    poiLocationManager = POILocationManager();
    print "POI: {} has the position {}".format(poiInfoManager.get_poi_name(), poiLocationManager.get_position(poiInfoManager.get_poi_name()));
    poiInfoManager.set_visited();
    print "NEXT POI: {}".format(poiInfoManager.next_poi());
    print "POI: {} has the position {}".format(poiInfoManager.get_poi_name(), poiLocationManager.get_position(poiInfoManager.get_poi_name()));
    print "NEXT POI: {}".format(poiInfoManager.next_poi());
    poiInfoManager.write_file();

if __name__ == '__main__':
    rospy.init_node('experiment_node', anonymous=True);
    infofilename = rospy.get_param('~infofilename', '/home/pal/default_rooms.json')
    soundfilename = rospy.get_param('~soundfilename', '/home/pal/default_sound.json')
    #test_POI_classes()
    try:
        rospy.loginfo("[EXPERIMENT_NODE] STARTED");
        my_logic_manager = ExperimentLogicManager(infofilename, soundfilename);
        rospy.spin();
    except KeyboardInterrupt:
        pass;