// kruskalMaze.hpp
#pragma once
#include "maze.hpp"

struct Bucket {
    Node* node = nullptr;
    int id = 0;
    Bucket(Node* _node, int _id) : node(_node), id(_id) {};
};

struct Edge {
    Node* NodeA = nullptr;
    Node* NodeB = nullptr;
    Edge(Node* _NodeA, Node* _NodeB) : NodeA(_NodeA), NodeB(_NodeB) {};

    bool operator== (const Edge &_Edge){
        Node* nA = _Edge.NodeA;
        Node* nB = _Edge.NodeB;
        if ((nA == NodeA && nB == NodeB) || (nB == NodeA && nA == NodeB))
            return true;
        else
            return false;
    }
};

class Kruscal : public Maze {
    private:
        std::vector<Bucket> bucketList = {};
        std::vector<Edge> edgeList = {};

    public:
        Kruscal() = delete;
        Kruscal(int w, int h) : Maze(w, h) {}

        // fill bucket and edge list
        void fillVectors() {
            Node* node;
            int id = 0;

            for (int y = 0; y < g_height; ++y) {
                for (int x = 0; x < g_width; ++x) {
                    node = grid.graph_matrix[y][x];
                    bucketList.push_back(Bucket(node,id));
                    for (int i = 0; i < (*node).edges.size(); ++i) {
                        Edge edge = Edge(node,(*node).edges[i]);
                        if (!(std::find(edgeList.begin(), edgeList.end(), edge) != edgeList.end()))
                            edgeList.push_back(edge);
                    }
                    ++id;
                }
            }
        }

        // fill the maze
        void fillMaze(){
            fillVectors();
                        
            // kruscal edge algorithm
            while (edgeList.size() > 0) { 

                // Choose a random edge
                std::uniform_int_distribution<> randomEdge(0, edgeList.size() - 1);
                int index = randomEdge(rng);
                auto edge = edgeList[index];

                // Remove edge from list
                edgeList.erase(edgeList.begin() + index);
                
                // Find bucket IDs
                int *IdNodeA, *IdNodeB;
                for (int i = 0; i < bucketList.size(); ++i) {
                    if (bucketList[i].node == edge.NodeA) {
                        IdNodeA = &bucketList[i].id;
                    } else if (bucketList[i].node == edge.NodeB) {
                        IdNodeB = &bucketList[i].id;
                    }
                }

                if ((*IdNodeA) != (*IdNodeB)) {
                    
                    // insert edge into maze
                    (*maze[(*edge.NodeA).posy][(*edge.NodeA).posx]).edges.push_back(maze[(*edge.NodeB).posy][(*edge.NodeB).posx]);
                    (*maze[(*edge.NodeB).posy][(*edge.NodeB).posx]).edges.push_back(maze[(*edge.NodeA).posy][(*edge.NodeA).posx]);
                    

                    // change bucket id's
                    int id = (*IdNodeB);
                    for (int i = 0; i < bucketList.size(); ++i) {
                        if (bucketList[i].id == id) {
                            bucketList[i].id = (*IdNodeA);
                        }
                    }
                }

            }
        }
};