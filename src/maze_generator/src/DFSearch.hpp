// DFSearch.hpp
#pragma once

#include <iostream>
#include <random>
#include <algorithm>
#include <stack>
#include "graph.hpp"

class DFS {
    private:
        std::vector<Node*> visited;
        std::stack<Node*> available;
        std::vector<std::vector<Node*>> maze = {};
        int start_x, start_y;
        int end_x, end_y;
        bool foundPath = false;
    public: 
        DFS() = delete;
        DFS(std::vector<std::vector<Node*>> _maze, int _start_x = 0, int _start_y = 0, int _end_x = 0, int _end_y = 0) 
            : maze(_maze), start_x(_start_x), start_y(_start_y), end_x(_end_x), end_y(_end_y) {
                makePath();
            };

        std::vector<Node*> printPath() {
            Node *node;
            std::vector<Node*> path = {};

            // Save the correct path
            Node *prev = maze[end_y][end_x];
            std::cout << "Path: (" << (*prev).posx << ", " << (*prev).posy << ")";
            path.push_back(prev);
            for (int i = visited.size() - 1; i >= 0; --i) {
                node = visited[i];
                for (int j = 0; j < (*prev).edges.size(); ++j) {
                    if ((*prev).edges[j] == node) {
                        path.push_back(node);
                        prev = node;
                        std::cout << " -> (" << (*node).posx << ", " << (*node).posy << ")";
                        break;
                    }
                }
            }
            std::cout << std::endl << std::endl;
            return path;
        }

        
        void makePath() {
            // Clear vector and stack
            visited.clear();
            for (int i = 0; i < available.size(); ++i)
                available.pop();

            // Choose the starting node
            Node* node = maze[start_y][start_x];
            available.push(node);
            
            // Recursive Depth First Search
            while (!available.empty()) {
                node = available.top(); 
                available.pop();
                if (node == maze[end_y][end_x])
                    break;
                search(node);
            }
        }

        void search(Node* &node) {
            if (!foundPath) {
                // Add node to visited vector
                if (!(std::find(visited.begin(), visited.end(), node) != visited.end()))
                    visited.push_back(node);
                
                // Available edges
                Node* edge;
                for (int i = 0; i < (*node).edges.size(); ++i) { 
                    edge = (*node).edges[i];
                    if (!(std::find(visited.begin(), visited.end(), edge) != visited.end())) {
                        if (edge == maze[end_y][end_x]) {
                            // Clear available stack
                            for (int i = 0; i < available.size(); ++i)
                                available.pop();
                            // Terminate the recursive algorithm
                            foundPath = true;
                            return;
                        }
                        available.push(edge);
                        search(edge);
                    }
                }
            }
            return;
        }
};