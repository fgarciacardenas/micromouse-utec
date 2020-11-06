#include "kruscalMaze.hpp"

int main() {
    #define width 16
    #define height 16

    Kruscal kruscal(width, height);
    kruscal.createMaze();
    kruscal.printGazebo("kruscal_maze");
    return 0;
}