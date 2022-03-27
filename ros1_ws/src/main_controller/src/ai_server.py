#!/usr/bin/env python3

import json
import os
from collections import Counter
from math import pi
from threading import Lock

import numpy as np
import rospy
from main_msgs.srv import ActorInfoS as ActorInfoSrv
from std_msgs.msg import String

# ------------------------------------------------------------------
#                     ACTOR INFO CLASS
#               (interface with Lorenzo's work)
# ------------------------------------------------------------------

class ActorInfo:

    def __init__(self):
        # Initialize Actor Info Server node
        rospy.init_node("ai_server")
        rospy.loginfo("Actor Info server node created")

        # Load personality of the robot (influences emotional state transitions)
        self.personality = rospy.get_param("/personality")
        
        # Read Emotional States Evolution Table to get the list of foreseen actions
        self.actor_actions = self.load_actor_actions(f"../resources/csv/Reactions.csv")

        # Load actor input source
        self.actor_input = rospy.get_param("/actor_input")
        
        # Load max scene duration
        self.max_num_of_calls = rospy.get_param("/max_scene_duration")
        
        if self.actor_input not in ["stdin", "world"]:  # load actor behaviour from json file
            try:
                f = open(f"../resources/actor_scenes/{self.actor_input}")
                self.scene = json.load(f)
                self.counter = 0
                f.close()
            except FileNotFoundError:
                rospy.logwarn(f"Could not find {self.actor_input}, switching to stdin")
                self.actor_input = "stdin"

        elif self.actor_input == "world":
            # Subscribe to ActorInfo publisher
            self.ai_sub = rospy.Subscriber("/my_actor_info", String, self.update_actor_info)
            self.up_to_date = False
            self.lock = Lock()
        
        self.actor_history = []
        self.actor_pos_x = self.actor_pos_y = self.actor_orientation = 0
        self.scenic_action = "none"
        self.end_scene = False
        self.timestamp = rospy.get_rostime()
        self.num_of_calls = 0
        
        # Start Actor Info Service
        service = rospy.Service("/get_actor_info", ActorInfoSrv, self.handle_ai)
        rospy.loginfo(f"Actor Info Service server has been started, input set to {self.actor_input}")

        self.rate = rospy.Rate(0.5)

        # Loop waiting for requests
        rospy.spin()

    # ------------------------------------------------------------------
    #                     STATE MACHINE HANDLING
    # ------------------------------------------------------------------

    def load_actor_actions(self, filename):
        reactions_matrix = np.genfromtxt(filename, delimiter=',', dtype=str) # read csv file into numpy array
        return reactions_matrix[1:,0].tolist()   # return first column, excluding first row; as list

    # ------------------------------------------------------------------
    #                             CALLBACKS
    # ------------------------------------------------------------------

    def handle_ai(self, req):
        """actor info service callback"""

        if self.actor_input == "stdin": # actor actions loaded from stdin

            self.scenic_action = ""
            while self.scenic_action not in self.actor_actions:
                self.scenic_action = input("Enter scenic action: ").lower()
            self.actor_pos_x = float(input("Insert actor posX: "))
            self.actor_pos_y = float(input("Insert actor posY: "))
            self.actor_orientation = float(input("Insert actor orientation (0 => facing the robot, 180 => giving the back to the robot, counterclockwise): ")) * pi / 180
            self.end_scene = (input("End Scene ? (y/n): ").upper()[0] == "Y")
        
        elif self.actor_input == "world":   # actor actions from world
            while len(self.actor_history) == 0 or not self.analyze_history():  # wait for something to happen
                self.rate.sleep()

        else:   # test scenes from json    
            self.actor_pos_x = self.scene[self.counter]["actorPosX"]
            self.actor_pos_y = self.scene[self.counter]["actorPosY"]
            # self.actor_orientation = self.scene[self.counter]["actorOrientation"]
            self.scenic_action = self.scene[self.counter]["scenicAction"]
            self.counter += 1
            if self.counter >= len(self.scene):
                self.end_scene = True
                self.counter = 0
            else:
                self.end_scene = False

        # Send Actor Info to Client
        return [self.actor_pos_x, self.actor_pos_y, self.actor_orientation, self.scenic_action, self.end_scene]


    def update_actor_info(self, msg):
        """async subscriber callback, add actor information (from world) into history"""

        self.lock.acquire()     # acquire lock
        
        # Load data from msg & push to history
        m = json.loads(msg.data)
        self.actor_history.append({
            "actor_pos_x" : round(m["actorPosX"], 2),
            "actor_pos_y" : round(m["actorPosY"], 2),
            "scenic_action" : m["scenicAction"].lower(),
            "time" : rospy.get_rostime()
        })
        
        self.lock.release()     # release lock


    def analyze_history(self):
        """analyze actor history, in order to spot what action the actor has done and decide whether to continue or not the scene
        
        returns true if something is ready to be processed, false otherwise"""
        
        self.lock.acquire()     # acquire lock

        now = rospy.get_rostime()
        
        # Consider only actions happened after last analysis
        history = [ elem for elem in self.actor_history if elem["time"] > self.timestamp ]

        # History Handling Logic
        self.actor_pos_x = history[-1]["actor_pos_x"]
        self.actor_pos_y = history[-1]["actor_pos_y"]
        data =  Counter(map(lambda x : x["scenic_action"], history[::-1]))
        self.scenic_action = data.most_common(1)[0][0]  # find most common scenic action, with priority to newest one

        rospy.loginfo(f">>> I analyzed {len(history)} actions, result is: {self.scenic_action}")
        
        if self.scenic_action == "none" and self.num_of_calls == 0:
            self.clear_history()
            self.lock.release()     # release lock
            return False # waiting for scene to begin

        self.num_of_calls += 1

        # End Scene Logic
        self.end_scene = (self.num_of_calls >= self.max_num_of_calls) or (self.num_of_calls > 0 and self.scenic_action == "none")

        self.timestamp = now

        self.lock.release()     # release lock
        
        return True

    
    def clear_history(self):
        self.actor_history = []


if __name__ == '__main__':
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    rospy.loginfo(">>> CURRENT WORKING DIRECTORY: " + os.getcwd())

    try:
        ActorInfo()
    except rospy.ROSInterruptException:
        pass
