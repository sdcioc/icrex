#! /usr/bin/python

# Author: Ciocirlan Stefan-Dan 19/09/2018
import json
import rospy
import std_msgs.msg


class ExperimentManualCommandManager:
    def __init__(self):
        self.command_pub = rospy.Publisher(
                    '/experiment/cmd',
                        std_msgs.msg.String,
                        latch=True, queue_size=5);
        self.rate = rospy.Rate(10);

    def send_stop_command(self):
        next_command = {};
        next_command['type'] = "STOP";
        next_command['manual'] = True;
        print "[INFO][MANUAL_COMMAND] sending STOP";
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
  
    def send_start_next_poi_command(self, poi_name):
        next_command = {};
        next_command['type'] = "START_NEXT_POI";
        next_command['manual'] = True;
        next_command['poi_name'] = poi_name;
        print "[INFO][MANUAL_COMMAND] sending START_NEXT_POI with poi: {}".format(poi_name);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();      

    def send_force_poi_command(self, poi_name):
        next_command = {};
        next_command['type'] = "FORCE_POI";
        next_command['manual'] = True;
        next_command['poi_name'] = poi_name;
        print "[INFO][MANUAL_COMMAND] sending FORCE_POI with poi: {}".format(poi_name);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();   

    def send_general_command(self, command_type):
        next_command = {};
        next_command['type'] = command_type;
        next_command['manual'] = True;
        print "[INFO][MANUAL_COMMAND] sending {}".format(command_type);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();
    
    def send_start_experiment_command(self):
        next_command = {};
        next_command['type'] = "START_EXPERIMENT";
        print "[INFO][MANUAL_COMMAND] sending START_EXPERIMENT";
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();       

    def send_manual_verify_door_command(self, door):
        next_command = {};
        next_command['type'] = "MANUAL_VERIFY_DOOR";
        next_command['door'] = door;
        print "[INFO][MANUAL_COMMAND] sending MANUAL_VERIFY_DOOR with dorr: {}".format(door);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    def send_manual_say_many_command(self):
        next_command = {};
        next_command['type'] = "MANUAL_SAY_MANY";
        print "[INFO][MANUAL_COMMAND] sending MANUAL_CHECK_DOOR with dorr: {}".format(door);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    def send_manual_check_people_command(self, people_number):
        next_command = {};
        next_command['type'] = "MANUAL_CHECK_PEOPLE";
        next_command['people_number'] = people_number;
        print "[INFO][MANUAL_COMMAND] sending MANUAL_CHECK_PEOPLE with number: {}".format(people_number);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    def send_manual_do_first_experiment_chosen_command(self, guessed_type):
        next_command = {};
        next_command['type'] = "MANUAL_DO_FIRST_EXPERIMENT_CHOSEN";
        next_command['guessed_type'] = guessed_type;
        print "[INFO][MANUAL_COMMAND] sending MANUAL_DO_FIRST_EXPERIMENT_CHOSEN with guessed_type: {}".format(guessed_type);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

    def send_manual_experiment_command(self, command_type):
        next_command = {};
        next_command['type'] = command_type;
        print "[INFO][MANUAL_COMMAND] sending {}".format(command_type);
        self.command_pub.publish(json.dumps(next_command));
        self.rate.sleep();

if __name__ == '__main__':
    rospy.init_node('experiment_manual_command', anonymous=True);
    #test_POI_classes()
    try:
        emc =  ExperimentManualCommandManager();
        while(True):
            print """[INFO][MANUAL_COMMAND] command types: \n 
                    0 : START_EXPERIMENT\n
                    1 : MANUAL_VERIFY_DOOR;\n
                    2 : MANUAL_SAY_MANY\n
                    3 : MANUAL_CHECK_PEOPLE\n
                    4 : MANUAL_DO_FIRST_EXPERIMENT_FIRST\n
                    5 : MANUAL_DO_FIRST_EXPERIMENT_SECOND\n
                    6 : MANUAL_DO_FIRST_EXPERIMENT_CHOSEN\n
                    7 : MANUAL_DO_SECOND_EXPERIMENT\n
                    8 : STOP\n
                    9 : START_NEXT_POI\n
                    10: GOTOPARENT\n
                    11: SILENT_GOTOPARENT\n
                    12: EXIT
                    """
            command_number = int(raw_input("Enter your command:\n"));
            if (command_number == 0):
                emc.send_start_experiment_command();
            elif (command_number == 1):
                door = int(raw_input("Enter DOOR OPEN 1/DOOR CLOSED 0:"));
                emc.send_manual_verify_door_command(door);
            elif (command_number == 2):
                emc.send_manual_say_many_command();
            elif (command_number == 3):
                people_number = int(raw_input("Enter NUMBER OF PEOPLE:"));
                emc.send_manual_check_people_command(people_number);
            elif (command_number == 6):
                guessed_type = int(raw_input("Enter guessed_type:"));
                emc.send_manual_do_first_experiment_chosen_command(guessed_type);
            elif (command_number == 8):
                emc.send_stop_command();
            elif (command_number == 9):
                poi_name = raw_input("Enter POI name");
                emc.send_start_next_poi_command(poi_name)
            elif (command_number == 12):
                poi_name = raw_input("Enter POI name");
                emc.send_force_poi_command(poi_name)
            else:
                command_type = "";
                if (command_number == 4):
                    command_type = "MANUAL_DO_FIRST_EXPERIMENT_FIRST"
                elif (command_number == 5):
                    command_type = "MANUAL_DO_FIRST_EXPERIMENT_SECOND"
                elif (command_number == 7):
                    command_type = "MANUAL_DO_SECOND_EXPERIMENT"
                elif (command_number == 10):
                    command_type = "GOTO_BACK_PARENT_POI"
                elif (command_number == 11):
                    command_type = "SILENT_GOTO_BACK_PARENT_POI"
                elif (command_number == 12):
                    break;
                else:
                    print "[ERORRE][MANUAL_COMMAND] command_number {}".format(command_number);
                emc.send_manual_experiment_command(command_type);
    except KeyboardInterrupt:
        pass;

