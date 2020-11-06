// graph.hpp
#pragma once
#include <iostream>
#include <vector>
#include <random>
#include <chrono>

// Generate random seed (chrono based)
std::mt19937 rng(std::chrono::high_resolution_clock::now().time_since_epoch().count());

// Node struct
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
        
        // Create a "grid" styled graph with given dimensions
        void createGrid(int _width, int _height){
            width = _width; height = _height;
            graph_matrix.resize(height, std::vector<Node *>(width));
            
            // Add required nodes
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    addNode(x, y);
                }
            }
            
            // Add required edges
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    Node* node = graph_matrix[y][x];
                    
                    // Edge insertion
                    if (y != height - 1) addEdge(node, graph_matrix[y + 1][x]);
                    if (y != 0) addEdge(node, graph_matrix[y - 1][x]);
                    if (x != width -1) addEdge(node, graph_matrix[y][x + 1]);
                    if (x != 0) addEdge(node, graph_matrix[y][x - 1]);
                }
            }
        }
        
        // Create a node
        void addNode(int posx, int posy) {
            Node* node = new Node(posx, posy);
            graph_matrix[posy][posx] = node;
            size_of_graph++;
        }
        
        // Create an edge
        void addEdge(Node* &A, Node* &B) {
            (*A).edges.push_back(B);
        }

        // Get functions
        unsigned int getSize() { return size_of_graph; }
        int getWidth() { return width; }
        int getHeight() { return height; }
};