import pandas as pd
import numpy as np
import time
import random

ACTIONS = ['up', 'down', 'left', 'right']
LENGTH = 3
N_STATES = LENGTH * LENGTH
TERMINAL = (0, 0)
EPSILON = None
# DURING THE DEVELOPMENT/IMPLEMENTATION PLEASE FEEL FREE TO CHANGE THIS VALUE
MAX_EPISODE = 500
GAMMA = None
ALPHA = None
START = None
HOLE1 = None
HOLE2 = None
TERMINAL = None
EPSILON = None
MAX_EPISODE = None
GAMMA = None
ALPHA = None
FIRST = True


def build_q_table():
    global N_STATES
    global ACTIONS
    table = pd.DataFrame(
        np.zeros((N_STATES, len(ACTIONS))),
        columns=ACTIONS
    )
    return table


def actor(observation, q_table):

    if np.random.uniform() < EPSILON:
        state_action = q_table.loc[observation, :]
        action = np.random.choice(
            state_action[state_action == np.max(state_action)].index)
    else:
        action = np.random.choice(ACTIONS)
    return action


def update_env(state, episode, step):
    view = np.array([['_ '] * LENGTH] * LENGTH)
    view[tuple(TERMINAL)] = '* '
    view[HOLE1] = 'X '
    view[HOLE2] = 'X '
    view[tuple(state)] = 'o '
    interaction = ''
    for v in view:
        interaction += ''.join(v) + '\n'

    # Following 3 line of code gives you an opportunity to see in jupyter how the agent learns
    # comment if you do not want to see. You can also modify  the transition time between agent states
    #########################################
    #########################################
    # clear_output(wait=True)
    # print(interaction)
    # time.sleep(0.025)
    #########################################
    #########################################


def init_env(robot_start, robot_end):
    global HOLE1
    global HOLE2
    global FIRST
    global START
    global TERMINAL
    HOLE1 = (1, 0)
    HOLE2 = (1, 1)
    TERMINAL = robot_end
    START = robot_start
    start = START
    FIRST = False
    return start, False


def get_env_feedback(state, action):
    reward = 0.
    end = False
    a, b = state

    if action == 'up':
        a -= 1
        if a < 0:
            a = 0
        next_state = (a, b)
        if next_state == TERMINAL:
            reward = 1.
            end = True
        elif (next_state == HOLE1) or (next_state == HOLE2):
            reward = -1.
            end = True
            
            
    elif action == 'down':
        a += 1
        if a >= LENGTH:
            a = LENGTH - 1
        next_state = (a, b)
        if next_state == TERMINAL:
            reward = 1.
            end = True
        if (next_state == HOLE1) or (next_state == HOLE2):
            reward = -1.
            end = True
            
            
    elif action == 'left':
        b -= 1
        if b < 0:
            b = 0
        next_state = (a, b)
        if next_state == TERMINAL:
            reward = 1.
            end = True
        if (next_state == HOLE1) or (next_state == HOLE2):
            reward = -1.
            end = True
            
            
    elif action == 'right':
        b += 1
        if b >= LENGTH:
            b = LENGTH - 1
        next_state = (a, b)
        if next_state == TERMINAL:
            reward = 1.
            end = True
        elif (next_state == HOLE1) or (next_state == HOLE2):
            reward = -1.
            end = True

    return next_state, reward, end


def playGame(q_table):
    maze_transitions = []
    waypoints = []
    state = (2, 0)  # START
    end = False
    LENGTH = 3
    a, b = state
    i = 0
    #waypoints.append(state)
    while not end:

        act = actor(a * LENGTH + b, q_table)
        print("step::", i, " action ::", act)
        maze_transitions.append(act)
        # waypoints.append(state)
        next_state, reward, end = get_env_feedback(state, act)
        state = next_state
        waypoints.append(state)
        a, b = state
        i += 1
    print("==> Game Over <==")
    return maze_transitions, waypoints


# Qlearning algoritm

def Qlearn(max_episode, robot_start, robot_end):
    MAX_EPISODE = max_episode
    q_table = build_q_table()
    episode = 0
    while episode < MAX_EPISODE:
        state, end = init_env(robot_start, robot_end)
        step = 0
        #update_env(state, episode, step)
        while not end:
            a, b = state

            act = actor(a * LENGTH + b, q_table)

            next_state, reward, end = get_env_feedback(state, act)

            na, nb = next_state

            q_predict = q_table.loc[a * LENGTH + b, act]

            if next_state != TERMINAL:
                # Qlearning algoritm
                ###################################################################
                q_target = reward + GAMMA * \
                    q_table.iloc[na * LENGTH + nb].max()
            else:
                q_target = reward
            q_table.loc[a * LENGTH + b, act] += ALPHA * (q_target - q_predict)
            state = next_state
            step += 1
            #update_env(state, episode, step)

            # if step > 30: # the same punish like presented in previous module. Choose to apply or not
            #   end = True

        episode += 1
    return q_table


def run_brain():

    LENGTH = 3
    N_STATES = LENGTH * LENGTH
    START = (2, 0)
    TERMINAL = (0, 0)
    global EPSILON
    global GAMMA
    global ALPHA
    global MAX_EPISODE
    EPSILON = .9
    # DURING THE DEVELOPMENT/IMPLEMENTATION PLEASE FEEL FREE TO CHANGE THIS VALUE
    MAX_EPISODE = 500
    GAMMA = .9
    ALPHA = .01
    robot_start = (2, 0)
    robot_end = (0, 0)

    q_table = Qlearn(MAX_EPISODE, robot_start, robot_end)
    print("====== Q TABLE AFTER LEARNING ======")
    print(q_table)
    print(" ")
    print("======ACTION TAKEN BY AGENT TO REACH THE GOAL======")
    maze_transitions, waypoints = playGame(q_table)
    #actions = Actions(maze_transitions)
    # print(waypoints)
    return waypoints
