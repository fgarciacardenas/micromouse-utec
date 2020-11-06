// graph.hpp
#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#pragma once

#include <iostream>
#include <vector>
#include <random>
#include <chrono>

// Generate random seed

//std::mt19937 rng(2); // const rng for testing
std::mt19937 rng(std::chrono::high_resolution_clock::now().time_since_epoch().count()); // chrono based rng (more portable compared to random_device)

// node struct
struct Node {
    int posx = 0, posy = 0;
    std::vector<Node*> edges;

    Node(int _posx = 0, int _posy = 0) : posx(_posx), posy(_posy) {};
};

class Graph {
    private:
        unsigned int size_of_graph = 0;
        int width = 0;
        int height = 0;

    public:
        std::vector<std::vector<Node*>> graph_matrix = {};
        
        // create a "grid" styled graph with given dimensions, as needed for a maze
        void createGrid(int _width, int _height){
            width = _width; height = _height;
            graph_matrix.resize(height, std::vector<Node *>(width));
            
            // add required nodes
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    addNode(x, y);
                }
            }
            
            // add required edges
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    Node* node = graph_matrix[y][x];
                    
                    // edge insertion logic
                    if (y != height - 1) addEdge(node, graph_matrix[y + 1][x]);
                    if (y != 0) addEdge(node, graph_matrix[y - 1][x]);
                    if (x != width -1) addEdge(node, graph_matrix[y][x + 1]);
                    if (x != 0) addEdge(node, graph_matrix[y][x - 1]);
                }
            }
        }
        
        // creates a node
        void addNode(int posx, int posy) {
            Node* node = new Node(posx, posy);
            graph_matrix[posy][posx] = node;
            size_of_graph++;
        }
        
        // creates an edge
        void addEdge(Node* &A, Node* &B) {
            (*A).edges.push_back(B);
        }

        // get functions
        unsigned int getSize() { return size_of_graph; }

        int getWidth() { return width; }

        int getHeight() { return height; }
};

#endif