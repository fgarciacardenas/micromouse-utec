#include <iostream>
#include "kruscalMaze.hpp"
#include "primMaze.hpp"
#include "DFSearch.hpp"

int main() {
    #define width 16
    #define height 16

    // Prim maze
    Prim prim16x16(width, height);
    prim16x16.createMaze();
    DFS prim16x16_dfs(prim16x16.getMaze(), 0, 0, width - 1, height - 1);
    std::cout << "Prim 16x16 ";
    prim16x16.printMaze("16x16Prim", prim16x16_dfs.printPath());
    prim16x16.printGazebo("prim_maze");

    // Kruscal maze
    Kruscal kruscal16x16(width, height);
    kruscal16x16.createMaze();
    DFS kruscal16x16_dfs(kruscal16x16.getMaze(), 0, 0, width - 1, height - 1);
    std::cout << "Kruscal 16x16 ";
    kruscal16x16.printMaze("16x16Kruscal", kruscal16x16_dfs.printPath());
    kruscal16x16.printGazebo("kruscal_maze");
    return 0;
}