import random
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.patches import Circle


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
        self.number_of_obstacles = 20
        self.obstacles = []
        self.is_path = True

    def RRT_generator(self):
        self.generate_obstacles(self.number_of_obstacles)
        self.random_q_init_and_goal()
        print(f"q_init: {self.q_init}")
        print(f"q_goal: {self.q_goal}")

        # Adding the initial configuration to the RRT list
        self.G = [self.q_init]
        # plt.figure()
        plt.axis([0, self.D[0], 0, self.D[1]])
        plt.title(
            f'An RRT with {self.number_of_obstacles} Obstacles and {self.K} Iterations')
        plt.scatter(self.q_init[0], self.q_init[1], color='black', s=10)
        plt.scatter(self.q_goal[0], self.q_goal[1], color='black', s=10)
        for k in range(self.K):
            print(k)
            self.q_rand = self.random_config(self.D)
            self.q_near = self.nearest_vertex(self.q_rand, self.G)
            self.q_new = self.new_config(self.q_near, self.q_rand, self.delta)
            if not self.check_collision(self.q_new):
                self.G.append(self.q_new)
                plt.pause(0.01)
                plt.plot([self.q_near[0], self.q_new[0]], [
                         self.q_near[1], self.q_new[1]], color='blue', markersize=1)
                plt.scatter(self.q_new[0], self.q_new[1], color='black', s=1)

                if not self.check_collision_line(self.q_new, self.q_goal):
                    self.G.append(self.q_goal)
                    plt.pause(0.01)
                    # import ipdb; ipdb.set_trace()
                    plt.plot([self.q_new[0], self.q_goal[0]], [
                             self.q_new[1], self.q_goal[1]], color='blue', markersize=1)
                    print(self.q_goal)
                    print(self.q_new)
                    print("Done!")
                    plt.pause(0.01)
                    break
        plt.show()

    def random_config(self, D):  # Generates a random position in the domain
        self.q_rand = [random.randrange(
            self.D[0]), random.randrange(self.D[1])]
        return self.q_rand

    # Finds the vertex in  that is closest to the given position
    def nearest_vertex(self, q_rand, G):
        smallest_dist = 100
        for i in self.G:
            if self.distance(i, self.q_rand) < smallest_dist:
                smallest_dist = self.distance(i, self.q_rand)
                self.q_near = i
        return self.q_near

    # Generates a new configuration in the tree
    def new_config(self, q_near, q_rand, delta):
        dist = self.distance(self.q_near, self.q_rand)
        if dist != 0:
            ratio_dist = self.delta / dist
            self.q_new = [((1 - ratio_dist) * self.q_near[0] + ratio_dist * self.q_rand[0]),
                          ((1 - ratio_dist) * self.q_near[1] + ratio_dist * self.q_rand[1])]
        return self.q_new

    def distance(self, point1, point2):
        dist = math.sqrt(
            pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2))
        return dist

    def distance_to_line(self, point1, point2, point3):
        dist = abs((point3[0] - point2[0]) * (point2[1] - point1[1]) - (point2[0] - point1[0]) * (
            point3[1] - point2[1])) / math.sqrt(pow(point3[0] - point2[0], 2) + pow(point3[1] - point2[1], 2))
        return dist

    def check_collision(self, point):
        collision = False
        for i in self.obstacles:
            if self.distance(i[0], point) < i[1]:
                collision = True
        return collision

    def check_collision_line(self, point1, point2):
        collision = False
        for i in self.obstacles:
            if self.distance_to_line(i[0], point1, point2) < i[1]:
                collision = True
        return collision

    def random_q_init_and_goal(self):
        self.q_init = [random.randrange(
            self.D[0]), random.randrange(self.D[1])]
        self.q_goal = [random.randrange(
            self.D[0]), random.randrange(self.D[1])]
        while self.check_collision(self.q_init):
            self.q_init = [random.randrange(
                self.D[0]), random.randrange(self.D[1])]
        while self.check_collision(self.q_goal):
            self.q_goal = [random.randrange(
                self.D[0]), random.randrange(self.D[1])]

    def generate_obstacles(self, number_of_obstacles):
        while number_of_obstacles:
            circle_center = (random.randrange(
                self.D[0]), random.randrange(self.D[1]))
            circle_radius = random.randrange(10)
            circle = plt.Circle(circle_center, circle_radius, color='r')
            plt.gcf().gca().add_artist(circle)
            self.obstacles.append([circle_center, circle_radius])
            number_of_obstacles = number_of_obstacles - 1


def main():
    random_tree = RandomTree(1500, 2, [100, 100])
    random_tree.RRT_generator()
    # print(random_tree)


if __name__ == "__main__":
    main()
