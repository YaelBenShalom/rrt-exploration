/// \file rrt.h
/// \brief Library for Rapidly-Exploring Random Tree implementation.

#ifndef RRT_INCLUDE_GUARD_HPP
#define RRT_INCLUDE_GUARD_HPP

#include<iostream>
#include<limits>
#include<random>
#include<vector>
#include<cmath>


/// \brief a class for RRT node
class Node {
    public:
        float x;
        float y;
        float cost = 0;
        std::vector<float> x_path;
        std::vector<float> y_path;
        Node* parent;

        Node(float _x, float _y) {
            x = _x;
            y = _y;
            parent = NULL;
            cost = 0;
        }
};


class Tree {
    public:
        Tree(const Tree&) = delete;

        virtual ~Tree() = default;



        Tree& operator=(const Tree&) = delete;

};