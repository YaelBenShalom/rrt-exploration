import matplotlib.pyplot as plt
import random
import math
import numpy as np


class RandomTree:
    def __init__(self, K, delta, D):
        # self.q_init = q_init                      # Initial configuration
        self.K = K  # Number of vertices in RRT
        self.delta = delta  # Incremental distance
        self.D = D  # The planning domain [x, y]
        self.G = []  # Initial RRT list
        self.q_rand = []
        self.q_near = []
        self.q_new = []
        self.q_init = []
        self.q_goal = []
        self.number_of_obstacles = 0
        self.obstacles = []

    def RRT_generator(self):
        self.generate_obstacles(self.number_of_obstacles)
        self.random_q_init_and_goal()
        self.G = [self.q_init]  # Adding the initial configuration to the RRT list
        plt.figure()
        plt.axis([0, self.D[0], 0, self.D[1]])
        plt.title(f'An RRT with {self.number_of_obstacles} Obstacles and {self.K} Iterations')
        plt.scatter(self.q_init[0], self.q_init[1], color='green', s=3)
        # plt.scatter(self.q_goal[0], self.q_goal[1], color='green', s=3)
        for k in range(self.K):
            self.q_rand = self.random_config(self.D)
            self.q_near = self.nearest_vertex(self.q_rand, self.G)
            self.q_new = self.new_config(self.q_near, self.q_rand, self.delta)
            if not self.check_collision(self.q_new):
                self.G.append(self.q_new)
                # plt.pause(1e-3)
                plt.plot([self.q_near[0], self.q_new[0]], [self.q_near[1], self.q_new[1]], color='blue', markersize=1)
                plt.scatter(self.q_new[0], self.q_new[1], color='black', s=1)
        plt.show()

    def random_config(self, D):  # Generates a random position in the domain
        self.q_rand = [random.randrange(self.D[0]), random.randrange(self.D[1])]
        return self.q_rand

    def nearest_vertex(self, q_rand, G):  # Finds the vertex in  that is closest to the given position
        smallest_dist = 100
        for i in self.G:
            if self.distance(i, self.q_rand) < smallest_dist:
                smallest_dist = self.distance(i, self.q_rand)
                self.q_near = i
        return self.q_near

    def new_config(self, q_near, q_rand, delta):  # Generates a new configuration in the tree
        dist = self.distance(self.q_near, self.q_rand)
        if dist != 0:
            ratio_dist = self.delta / dist
            self.q_new = [((1 - ratio_dist) * self.q_near[0] + ratio_dist * self.q_rand[0]),
                          ((1 - ratio_dist) * self.q_near[1] + ratio_dist * self.q_rand[1])]
        return self.q_new

    def distance(self, point1, point2):
        dist = math.sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2))
        return dist

    def check_collision(self, point):
        collision = False
        for i in self.obstacles:
            if self.distance(i[0], point) < i[1]:
                collision = True
        return collision

    def random_q_init_and_goal(self):
        self.q_init = [random.randrange(self.D[0]), random.randrange(self.D[1])]
        self.q_goal = [random.randrange(self.D[0]), random.randrange(self.D[1])]
        while self.check_collision(self.q_init):
            self.q_init = [random.randrange(self.D[0]), random.randrange(self.D[1])]
        while self.check_collision(self.q_goal):
            self.q_goal = [random.randrange(self.D[0]), random.randrange(self.D[1])]

    def generate_obstacles(self, number_of_obstacles):
        while number_of_obstacles:
            circle_center = (random.randrange(self.D[0]), random.randrange(self.D[1]))
            circle_radius = random.randrange(10)
            circle = plt.Circle(circle_center, circle_radius, color='r')
            plt.gcf().gca().add_artist(circle)
            self.obstacles.append([circle_center, circle_radius])
            number_of_obstacles = number_of_obstacles - 1


def main():
    random_tree = RandomTree(1000, 1, [100, 100])
    random_tree.RRT_generator()
    # print(random_tree)


if __name__ == "__main__":
    main()
