#!/usr/bin/env python

"""
Training script for the Q learning algorithm.
Execute as a standalone python script to train the Q table

REQUIRES:
The robot_emulator_server.py to be running
env.py, csv_interface.py and robot_stepper_clinet.py in the same directory

OUTPUTS:
The fully trained Q table and the relevant policy on screen and as .csv files in the same directory
If run through launch file, .csv files go to tejas/resources directory, else .csv files go to same directory as script

https://medium.com/swlh/introduction-to-reinforcement-learning-coding-sarsa-part-4-2d64d6e37617
"""

import sys
import numpy as np
import random
import env
import robot_stepper_client
import csv_interface

if len(sys.argv) == 6:
    q_table_file = sys.argv[1]
    q_policy_file = sys.argv[2]
    total_episodes = int(sys.argv[3])
else:
    q_table_file = "SARSA_table.csv"
    q_policy_file = "SARSA_policy.csv"
    total_episodes = 100          
    
print("Running for {} episodes".format(total_episodes))

# Hyperparameters
max_steps = 100               
learning_rate = 0.8           
gamma = 0.1                   

# Exploration/Exploitation
epsilon = 1.0                 
max_epsilon = 1.0             
min_epsilon = 0.01            
decay_rate = 0.01            

# Environment variables
action_size = env.nA
state_size = env.nS
qtable = np.zeros((state_size, action_size))
qtable_new = np.zeros((state_size, action_size))

state_list = range(0, state_size)
action_list = range(0, action_size)

# Q training algorithm
for episode in range(total_episodes):
    state = random.choice(state_list)
    step = 0
    done = False
    total_rewards = 0
    explored = 0
    exploited = 0
    max_variation = 0

    exp_exp_tradeoff = random.uniform(0, 1)
    if exp_exp_tradeoff > epsilon:
        action = np.argmax(qtable[state,:])
        exploited += 1
    else:
        action = random.choice(action_list)
        explored += 1


    for step in range(max_steps):
        
        new_state, reward, done, info = robot_stepper_client.step(state, action, "False")
        
        exp_exp_tradeoff = random.uniform(0, 1)
        if exp_exp_tradeoff > epsilon:
            new_action = np.argmax(qtable[new_state,:])
            exploited += 1
        else:
            new_action = random.choice(action_list)
            explored += 1
        
        
        qtable_new[state, action] = qtable[state, action] + learning_rate * (reward + gamma * (qtable[new_state,new_action ]) - qtable[state, action])
        delta = abs (qtable_new[state, action] - qtable[state, action])
        max_variation = max(max_variation, delta)
        qtable[state, action] = qtable_new[state, action]

        state = new_state
        action = new_action
        total_rewards += reward

        if done: 
            break
        
    if episode%10 == 0: 
        print("Episode #{}".format(episode))
        print("Exploited : {} Explored : {} Max_Varaition : {} Total_Reward: {}".format(exploited, explored, max_variation, total_rewards))
        print(qtable)
        print("")


    epsilon = min_epsilon + (max_epsilon - min_epsilon)*np.exp(-decay_rate*episode) 

# Display and save results
print("")
print("Q Table :")
print(qtable)
print("")

policy = []
for row in qtable:
    policy.append(np.argmax(row))
print("Extracted policy :")
print(policy)

print("Saving sarsa policy to : {}".format(q_policy_file))
csv_interface.writeQTableToCsv(q_table_file, qtable)
print("Saving sarsa table to : {}".format(q_table_file))
csv_interface.writePolicyArrayToCsv(q_policy_file, policy)
