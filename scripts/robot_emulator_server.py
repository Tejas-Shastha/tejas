#!/usr/bin/env python

"""
SERVER OF THE SERVICE :
Service emulates the robot IE: Given a state and an action, picks a random valid next state
and returns corresponding (state_next, reward, done, info) tuple.

Run this node as a standalone without any enclosing namespace

@params
Request
int64 state         - Current state
int64 action        - Selected action
string terminating  - True if using terminating model, else False
Response
int64 state_next    - Randomly chosen valid next state
int64 reward        - Immedeate reward
bool terminal       - Is done?
string info         - Additional info

"""

import random
import rospy
from tejas.srv import EnvStep, EnvStepResponse
import env

TERM_P = []
NON_TERM_P = []


def reshape_reward(reward, positive, negative):
    return positive if reward == -1 else negative


def emulate_step(state, action, terminating):
    if terminating == "False" or terminating == "false":
        term = False
    elif terminating == "True" or terminating == "true":
        term = True
    else:
        print("Faulty EnvStepRequest.terminating")
        return 0, 0, 0, "Faulty EnvStepRequest.terminating"

    P = TERM_P if term else NON_TERM_P
    p_distribution = P[state][action]       # Stochastic response for given state and action = 3 tuples of 4 elements [s_, r, t, i]
    random_choice = p_distribution[random.choice((0, 1, 2))]    # Randomly determine a single response = 1 tuple of 4 elements [s_, r, t, i]
    state_next = random_choice[1] 
    reward = random_choice[2] if term else reshape_reward(random_choice[2], 3, -3)
    terminal = random_choice[3]
    info = ""
    return state_next, reward, terminal, info

def step_action(req):
    print("Received request : S={} A={} T= {}".format(req.state, req.action, req.terminating))
    state_next, reward, terminal, info = emulate_step(req.state, req.action, req.terminating)
    print("Returning S_={} R={} T={} I={}".format(state_next, reward, terminal, info))
    return EnvStepResponse(state_next, reward, terminal, info)


def robot_emulator_service():
    rospy.init_node("robot_emulator_server")
    emulator_service = rospy.Service("robot_emulator_service", EnvStep, step_action)
    
    global TERM_P, NON_TERM_P
    env.buildP(True)
    TERM_P = env.P
    env.scrubP()
    env.buildP(False)
    NON_TERM_P = env.P
    env.scrubP()    
    
    print("Emulator running")
    rospy.spin()


if __name__ == "__main__":
    robot_emulator_service()
