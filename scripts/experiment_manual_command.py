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

if __name__ == '__main__':
    rospy.init_node('experiment_manual_command', anonymous=True);
    #test_POI_classes()
    try:
        emc =  ExperimentManualCommandManager();
        while(True):
            print """[INFO][MANUAL_COMMAND] command types: \n 
                    0 : START_EXPERIMENT\n
                    1 : STOP;\n
                    2 : START_NEXT_POI\n
                    3 : NEXT_POI\n
                    4 : FINISH_EXPERIMENT\n
                    5 : GOTO_PARENT_POI\n
                    6 : VERIFY_DOOR\n
                    7 : GOTO_POI\n
                    8 : CHECK_PEOPLE\n
                    9 : GOTO_BACK_PARENT_POI\n
                    10 : DO_EXPERIMENT\n
                    11 : EXIT\n
                    """
            command_number = int(raw_input("Enter your command:\n"));
            if (command_number == 0):
                emc.send_start_experiment_command();
            elif (command_number == 1):
                emc.send_stop_command();
            elif (command_number == 2):
                poi_name = raw_input("Enter POI name");
                emc.send_start_next_poi_command(poi_name)
            else:
                command_type = "";
                if (command_number == 3):
                    command_type = "NEXT_POI"
                elif (command_number == 4):
                    command_type = "FINISH_EXPERIMENT"
                elif (command_number == 5):
                    command_type = "GOTO_PARENT_POI"
                elif (command_number == 6):
                    command_type = "VERIFY_DOOR"
                elif (command_number == 7):
                    command_type = "GOTO_POI"
                elif (command_number == 8):
                    command_type = "CHECK_PEOPLE"
                elif (command_number == 9):
                    command_type = "GOTO_BACK_PARENT_POI"
                elif (command_number == 10):
                    command_type = "DO_EXPERIMENT"
                elif (command_number == 11):
                    break;
                else:
                    print "[ERORRE][MANUAL_COMMAND] command_number {}".format(command_number);
                emc.send_general_command(command_type);
    except KeyboardInterrupt:
        pass;

