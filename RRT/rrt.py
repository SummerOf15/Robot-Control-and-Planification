"""
RRT_2D
@author: huiming zhou

Modified by David Filliat
"""

import os
import sys
import math
import numpy as np
import plotting, utils
import env
import matplotlib.pyplot as plt

# parameters
showAnimation = True

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, environment, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = environment
        self.plotting = plotting.Plotting(self.env, s_start, s_goal)
        self.utils = utils.Utils(self.env)

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):

        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len:
                    self.new_state(node_new, self.s_goal)
                    return self.extract_path(node_new), i

        return None, self.iter_max

    def generate_random_node(self, goal_sample_rate):
        if np.random.random() < goal_sample_rate:
            return self.s_goal
        
        delta = self.utils.delta

        random_node=Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                    np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))
        while(self.utils.is_inside_obs(random_node)):
            random_node=Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                    np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))
        
        return random_node
        

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
        

def get_path_length(path):
    """
    Compute path length
    """
    length = 0
    for i,k in zip(path[0::], path[1::]):
        length += math.dist(i,k)
    return length


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node
    environment = env.Env()

    y1_plot=[]
    y2_plot=[]

    mean_iter=[]
    mean_length=[]
    step_len_list=[1,2,4,8]
    for step_len in step_len_list:
        for i in range(100):
            rrt = Rrt(environment, x_start, x_goal, step_len, 0.10, 10000)
            path, nb_iter = rrt.planning()

            mean_iter.append(nb_iter)
            mean_length.append(get_path_length(path))
            if path:
                # print('Found path in ' + str(nb_iter) + ' iterations, length : ' + str(get_path_length(path)))
                if showAnimation:
                    rrt.plotting.animation(rrt.vertex, path, "RRT", True)
                    plotting.plt.show()
            else:
                print("No Path Found in " + str(nb_iter) + " iterations!")
                if showAnimation:
                    rrt.plotting.animation(rrt.vertex, [], "RRT", True)
                    plotting.plt.show()
        y1_plot.append(sum(mean_iter)/len(mean_iter))
        y2_plot.append(sum(mean_length)/len(mean_length))
        print("step_len={},mean iter={},mean length={}".format(step_len,sum(mean_iter)/len(mean_iter),sum(mean_length)/len(mean_length)))

    plt.subplot(121)
    plt.plot(step_len_list,y1_plot,"r-o",label="mean_iter")
    
    plt.xlabel("step len")
    plt.ylabel("iter number")
    plt.subplot(122)
    plt.plot(step_len_list,y2_plot,"b-o",label="mean_length")
    plt.xlabel("step len")
    plt.ylabel("path length")
    plt.show()

if __name__ == '__main__':
    # main()
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node
    environment = env.Env2()
    p_list=[i*0.1 for i in range(11)]
    for p in p_list:
        rrt = Rrt(environment, x_start, x_goal, 2, 0.10, 10000)
        path, nb_iter = rrt.planning()
        if path:
            print('Found path in ' + str(nb_iter) + ' iterations, length : ' + str(get_path_length(path)))
            if showAnimation:
                rrt.plotting.animation(rrt.vertex, path, "RRT", True)
                plotting.plt.savefig("p_{}.png".format(p))
                # plotting.plt.show()
        else:
            print("No Path Found in " + str(nb_iter) + " iterations!")
            if showAnimation:
                rrt.plotting.animation(rrt.vertex, [], "RRT", True)
                plotting.plt.savefig("p_{}.png".format(p))
                # plotting.plt.show()
        