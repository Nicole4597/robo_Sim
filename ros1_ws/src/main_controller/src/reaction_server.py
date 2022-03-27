#!/usr/bin/env python3

import rospy
from main_msgs.srv import Reaction
import numpy as np

REACTION_STATES = np.genfromtxt("../resources/csv/Reactions.csv", delimiter=',', dtype=str) [1:,0].tolist()

# ------------------------------------------------------------------
#                       REACTION SERVER CALLBACK
# ------------------------------------------------------------------

def handle_reaction_request(req):
    pause_after = 1 if req.reaction != "none" else 0        # pause after the action is over
    
    if req.reaction not in REACTION_STATES:
        rospy.logwarn(f">>> UNFORSEEN REACTION: {req.reaction}")
        return 0,0,0,0,0,[],0
    
    approach_dir, approach_dist, approach_speed = should_approach_actor(req.reaction)
    rotate_dir, rotate_speed = should_rotate(req.reaction)
    actions = additional_actions(req.reaction)
    
    rospy.loginfo(f">>> REACTING '{req.reaction}'")
    rospy.loginfo(f"approach: {approach_dir}, rotate: {rotate_dir}\nactions: {actions}")
    
    return approach_dir, approach_dist, approach_speed, rotate_dir, rotate_speed, actions, pause_after


# ------------------------------------------------------------------
#                           PRIVATE METHODS
# ------------------------------------------------------------------

def should_approach_actor(reaction :str) -> tuple:
    """returns tuple (approach, dist_percentage, speed)"""
    default = (0, 0.5, 0.5) # default values
    
    approach_dict = {
        "sharing_happiness"  : ( 1, 0.75, 0.75),
        "surprise"           : ( 1, 0.5 , 1   ),
        "sharing_fear"       : ( 1, 0.75, 0.3 ),
        "sharing_sadness"    : ( 1, 0.75, 0.3 ),
        "attack"             : ( 1, 0.9 , 0.9 ),
        "intimidate"         : ( 1, 0.9 , 0.6 ),
        "scolding"           : ( 1, 0.75, 0.65),
        "satisfied"          : (-1, 0.5 , 0.5 ),
        "disappointment"     : (-1, 0.75, 0.4 ),
        "running_away"       : (-1, 0.9 , 1   ),
        "grudge"             : (-1, 0.75, 0.5 ),
    }

    return approach_dict[reaction] if reaction in approach_dict else default


def should_rotate(reaction :str) -> tuple:
    rotation_dir = 0
    rotation_speed = 1

    if reaction in ["sharing_happiness", "sharing_fear", "sharing_sadness", "attack", "intimidate", "disbelief", "astonishment"]:
        rotation_dir = 1
    
    if reaction in ["satisfied", "disappointment", "running_away", "grudge"]:
        rotation_dir = -1
    
    return rotation_dir, rotation_speed


def additional_actions(reaction :str) -> list:
    actions = { # reaction -> actions
        "sharing_happiness":    ["self_rotate", "dangling_eyes"],
        "happy":                ["dangling_eyes", "dangling_body", "self_rotate"],
        "satisfied":            ["dangling_eyes", "dangling_body"],
        "surprise":             ["bending_back", "raising_eyes"],
        "disappointment":       ["raising_eyes"],
        "sharing_fear":         ["crossing_eyes", "rocking_back_forth"],
        "astonishment":         ["forth_and_back"],
        "running_away":         ["eyes_up"],
        "sharing_sadness":      ["bow_slow", "crossing_eyes_slow"],
        "attack":               ["bow"],
        "scolding":             ["crossing_eyes"],
        "intimidate":           ["bow"],
        "grudge":               ["eyes_up"],
        "disbelief":            ["quarter_rotate", "eyes_up", "eyes_reset", "half_rotate", "eyes_up", "eyes_reset", "quarter_rotate", "bending_back", "eyes_up"],
        "none":                 [],
    }

    return actions[reaction]

# ------------------------------------------------------------------
#                               MAIN
# ------------------------------------------------------------------

if __name__ == '__main__':

    # Initialize Reaction Server node
    rospy.init_node("reaction_server")
    rospy.loginfo("Reaction server node created")

    # Start Reaction Service
    service = rospy.Service("/get_reaction", Reaction, handle_reaction_request)
    rospy.loginfo("Reaction Service server has been started")

    # Loop waiting for requests
    rospy.spin()
