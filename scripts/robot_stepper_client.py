#!/usr/bin/env python

"""
CLIENT OF THE SERVICE :
Service emulates the robot IE: Given a state and an action, picks a random valid next state
and returns corresponding (state_next, reward, done, info) tuple.

This is intended to be included in other scripts where the .step() function performs a service call
Terminal execution performs a test call based on runtime arguments

@params of service
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

import sys
import rospy
from tejas.srv import EnvStep



def step(state, action, terminating="False"):
    """
    @params
    state - input state
    action - input action
    terminating - use terminating model if true, else use non terminating model
    s_ - next state
    r - reward
    t - if terminal
    i - info

    @description
    Steps the robot from given state with given action and returns the result
    Invokes a service to acquire the results
    """
    rospy.wait_for_service("robot_emulator_service")
    try:
        emulator_service_client = rospy.ServiceProxy("robot_emulator_service", EnvStep)
        response = emulator_service_client(state, action, terminating)
        return response.state_next, response.reward, response.terminal, response.info
    except rospy.ServiceException as error:
        print("Service call failed :", error)


def usage():
    """
    Defines the usage when called directly
    """
    return "{} [state action terminating]".format(sys.argv[0])


if __name__ == "__main__":
    if len(sys.argv) == 4:
        STATE = int(sys.argv[1])
        ACTION = int(sys.argv[2])
        TERM = sys.argv[3]
    else:
        print(usage())
        sys.exit(1)
    string = TERM
    print("Requesting {} {} {}".format(STATE, ACTION, string))
    s_, r, t, i = step(STATE, ACTION, string)
    print("S_={} R={} T={} I={} ".format(s_, r, t, i))