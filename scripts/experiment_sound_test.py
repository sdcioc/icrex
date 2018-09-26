#! /usr/bin/python

# Author: Ciocirlan Stefan-Dan 25/09/2018
import rospy
import json
#sound
import subprocess
import rospkg


class SoundManager:
    def __init__(self):
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
        command = command + language + "_inital.wav";
        subprocess.check_output(command.split());


if __name__ == '__main__':
    rospy.init_node('experiment_sound_test_node', anonymous=True);
    try:
        rospy.loginfo("[EXPERIMENT_SOUND_TEST_NODE] STARTED");
        my_sound_manager = SoundManager();
        my_sound_manager.play_first(0);
        rospy.sleep(2);
        my_sound_manager.play_first(1);
        rospy.sleep(2);
        my_sound_manager.play_second(0);
        rospy.sleep(2);
        my_sound_manager.play_second(1);
        rospy.sleep(2);
    except KeyboardInterrupt:
        pass;