#!/usr/bin/env python
import random
import rospy
from tejas.srv import EnvStep, EnvStepResponse
import env


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


    env.buildP(term)
    p_distribution = env.P[state][action]       # Stochastic response for given state and action = 3 tuples of 4 elements [s_, r, t, i]
    random_choice = p_distribution[random.choice((0, 1, 2))]    # Randomly determine a single response = 1 tuple of 4 elements [s_, r, t, i]
    state_next = random_choice[1] 
    reward = random_choice[2] if term else reshape_reward(random_choice[2], 3, -3)
    terminal = random_choice[3]
    info = ""
    env.scrubP()
    return state_next, reward, terminal, info

def step_action(req):
    print("Received request : S={} A={} T= {}".format(req.state, req.action, req.terminating))
    state_next, reward, terminal, info = emulate_step(req.state, req.action, req.terminating)
    print("Returning S_={} R={} T={} I={}".format(state_next, reward, terminal, info))
    return EnvStepResponse(state_next, reward, terminal, info)


def robot_emulator_service():
    rospy.init_node("robot_emulator_server")
    emulator_service = rospy.Service("robot_emulator_service", EnvStep, step_action)
    print("Emulator running")
    rospy.spin()


if __name__ == "__main__":
    robot_emulator_service()


# Backup : Alternate s_, r, t rules :

# FORCE_F_1_2_THRESH = 0.5
# FORCE_F_2_3_THRESH = 1.0

# ROTATE_FEED = 1
# ROTATE_DONT_FEED = 2

# # UPPER_FEED_ANGLE_THRESH = 140 # was 140
# NUMBER_OF_ARM_SUB_STATES = 5

# # LOWER_ANGLE_THRESH = 85

# ACTION_DOWN = 0
# ACTION_STAY = 1
# ACTION_UP = 2

# FORCE_SAYS_DOWN = 0
# FORCE_SAYS_STAY = 1
# FORCE_SAYS_UP = 2

# INTERMEDEATE_POSITIVE_REWARD = -1
# INTERMEDEATE_NEGATIVE_REWARD = -3




    # # General states rewards
    # if force == action:
    #     reward = INTERMEDEATE_POSITIVE_REWARD
    # else:
    #     reward = INTERMEDEATE_POSITIVE_REWARD
    
    # # Border states rewards modification
    # if arm == 0 and (force == FORCE_SAYS_DOWN or force == FORCE_SAYS_STAY):
    #     if action == ACTION_STAY:
    #         reward = INTERMEDEATE_POSITIVE_REWARD
    #     else:
    #         reward = INTERMEDEATE_NEGATIVE_REWARD
    # if arm == NUMBER_OF_ARM_SUB_STATES-1 and (force == FORCE_SAYS_STAY or force == FORCE_SAYS_UP):
    #     if action == ACTION_STAY:
    #         reward = INTERMEDEATE_POSITIVE_REWARD
    #     else:
    #         reward = INTERMEDEATE_NEGATIVE_REWARD

    
    # if action == ACTION_DOWN:
    #     if arm == 0:
    #         arm_new = 0
    #     else:
    #         arm_new = arm - 1
    # if action == ACTION_STAY:
    #     arm_new = arm
    # if action == ACTION_UP:
    #     if arm == NUMBER_OF_ARM_SUB_STATES-1:
    #         arm_new = arm
    #     else:
    #         arm_new = arm +1
    
    # forces_list = (0, 1, 2)
    # force_new = random.choice(forces_list)

    # state_next = (3 * arm_new) + force_new

    # if arm_new == NUMBER_OF_ARM_SUB_STATES-1:
    #     terminal = True
    # else:
    #     terminal = False  
    
    # info = ""