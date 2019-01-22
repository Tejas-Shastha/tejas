#!/usr/bin/env python

import sys
import rospy
from tejas.srv import EnvStep



def step(state, action, terminating="True"):
    """
    @params
    state - input state
    action - input action
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