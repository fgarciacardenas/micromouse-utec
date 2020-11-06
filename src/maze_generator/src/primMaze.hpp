// primMaze.hpp
#pragma once
#include "maze.hpp"

class Prim : public Maze{
    public:
        Prim() = delete;
        Prim(int w, int h) : Maze(w, h) {}

        // fill the maze
        void fillMaze(){
            Node* start = startingNode();
            Node* next = chooseNext(start);
            while (next != nullptr) {
                (*maze[(*start).posy][(*start).posx]).edges.push_back(maze[(*next).posy][(*next).posx]);
                (*maze[(*next).posy][(*next).posx]).edges.push_back(maze[(*start).posy][(*start).posx]);
                start = next;
                next = chooseNext(start);
            }
        }

        // recursively choose next node
        Node* chooseNext(Node* &node, int recursion = 1) {
            // Add node to prev_node vector
            if (!(std::find(prev_nodes.begin(), prev_nodes.end(), node) != prev_nodes.end()))
                prev_nodes.push_back(node);
            
            // Available edges
            std::vector<Node*> available;
            Node* edge;
            
            for (int i = 0; i < (*node).edges.size(); ++i) { 
                edge = (*node).edges[i];
                if (!(std::find(prev_nodes.begin(), prev_nodes.end(), edge) != prev_nodes.end()))
                    available.push_back(edge);
            }
            
            // Generate distribution in range [0, edge_list_size]
            if (!available.empty()) {
                std::uniform_int_distribution<> randomEdge(0, available.size()-1);  
                return available[randomEdge(rng)];
            }

            if ((prev_nodes.size() - recursion) > 0) {
                node = prev_nodes[prev_nodes.size() - recursion];
                recursion++;
                return chooseNext(node, recursion);
            }
            else
                return nullptr;
        }
};