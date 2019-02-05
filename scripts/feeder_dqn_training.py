#!/usr/bin/env python

import random
import env
import robot_stepper_client
import csv_interface
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
from keras.models import model_from_json

GAMMA = 0.1
LEARNING_RATE = 0.001

MEMORY_SIZE = 1000000
BATCH_SIZE = 20

EXPLORATION_MAX = 1.0
EXPLORATION_MIN = 0.1
EXPLORATION_DECAY = 0.995

class DQNSolver:

    def __init__(self, observation_space, action_space):
        self.exploration_rate = EXPLORATION_MAX

        self.action_space = action_space
        self.memory = deque(maxlen=MEMORY_SIZE)
        self.observation_space = observation_space
    
    def build_fresh(self):
        self.model = Sequential()
        self.model.add(Dense(24, 
                        input_dim=self.observation_space, # 1D = nS
                        activation="relu"))
        self.model.add(Dense(24, activation="relu"))
        self.model.add(Dense(24, activation="relu"))
        self.model.add(Dense(self.action_space, activation="linear"))
        self.model.compile(loss="mse", optimizer=Adam(lr=LEARNING_RATE))
        self.train_mode = True

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() < self.exploration_rate and self.train_mode:
            return random.randrange(self.action_space)
        q_values = self.model.predict(state)
        return np.argmax(q_values[0])

    def experience_replay(self):
        if len(self.memory) < BATCH_SIZE:
            return
        batch = random.sample(list(self.memory), BATCH_SIZE)
        for state, action, reward, state_next, terminal in batch:
            q_update = reward
            if not terminal:
                q_update = (reward + GAMMA * np.amax(self.model.predict(state_next)[0]))
            q_values = self.model.predict(state)
            q_values[0][action] = q_update
            self.model.fit(state, q_values, verbose=0)
        self.exploration_rate *= EXPLORATION_DECAY
        self.exploration_rate = max(EXPLORATION_MIN, self.exploration_rate)

    def save_model(self):
        model_json = self.model.to_json()
        with open("feeder_dqn.json","w") as json_file:
            json_file.write(model_json)
        self.model.save_weights("feeder_dqn.h5")
        print("Saved model and weights")

    def load_model(self):
        json_file = open('feeder_dqn.json', 'r')
        loaded_model_json = json_file.read()
        json_file.close()
        loaded_model = model_from_json(loaded_model_json)
        # load weights into new model
        loaded_model.load_weights("feeder_dqn.h5")
        print("Loaded model from disk")
        self.model = loaded_model
        self.model.compile(loss="mse", optimizer=Adam(lr=LEARNING_RATE))
        self.train_mode = False
        return loaded_model
    
    def getQTable(self):
        policy = []
        q_table=[]
        for state in range(env.nS):
            state_vector = np.zeros(self.observation_space)
            state_vector[state] = 1
            state_vector = np.reshape(state_vector,[1,self.observation_space])
            q_values = self.model.predict(state_vector)
            print("S:{} Q:{}".format(state,q_values[0]))
            policy.append(np.argmax(q_values[0]))
            q_table.append(q_values)
               
        print(policy)
        return q_table, policy

    
    # def savePolicy(self):
    #     qtable = np.zeros((self.observation_space, self.action_space))
    #     for state in range(self.observation_space):
    #         qtable[state] = self.model.predict()



def feeder():
    observation_space = env.nS
    action_space = env.nA
    state_list = range(0, observation_space)
    action_list = range(0, action_space)

    dqn_solver = DQNSolver(observation_space, action_space)
    dqn_solver.build_fresh()
    states_visited = np.zeros(dqn_solver.observation_space)
    episode = 0
    
    while episode in range(200):
        episode += 1
        state = random.choice(state_list)
        
        run = 0
        total_reward = 0
        while run in range(100):
            run += 1
            
            state_vector = np.zeros(dqn_solver.observation_space)
            state_vector[state] = 1
            state_vector = np.reshape(state_vector,[1,dqn_solver.observation_space])

            states_visited[state] += 1
            action = dqn_solver.act(state_vector)
            state_next, reward, terminal, info = robot_stepper_client.step(state, action, "False")
            state_next = np.reshape(state_next,[1,])
            reward = reward # negate rewards since model has all negative rewards

            if dqn_solver.train_mode:
                state_next_vector = np.zeros(dqn_solver.observation_space)
                state_next_vector[state_next] = 1
                state_next_vector = np.reshape(state_next_vector,[1,dqn_solver.observation_space])
                dqn_solver.remember(state_vector, action, reward, state_next_vector, terminal)
            state = state_next
            total_reward += reward
            if dqn_solver.train_mode:
                dqn_solver.experience_replay()

        print("")
        print("")
        dqn_solver.getQTable()
        print("Ep:{} Tot_rew:{} eps.{}".format(episode, total_reward, dqn_solver.exploration_rate))
        # print("States visited :", states_visited)

    if dqn_solver.train_mode:
        dqn_solver.save_model()
    print("")
    print("Q Table: ")
    dqn_solver.getQTable()
    

if __name__=="__main__":
    feeder()

        


