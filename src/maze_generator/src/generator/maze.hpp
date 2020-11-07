// maze.hpp
#pragma once
#include "graph.hpp"
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <random>
#include <algorithm>
#include <vector>

#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

std::string get_current_dir() {
   char buff[FILENAME_MAX]; //create string buffer to hold path
   GetCurrentDir( buff, FILENAME_MAX );
   std::string current_working_dir(buff);
   return current_working_dir;
}

class Maze{
    protected:
        Graph grid;
        std::vector<std::vector<Node*>> maze = {};
        int g_width = 0;
        int g_height = 0;
        std::vector<Node*> prev_nodes;
    
    public:
        Maze() = delete;
        Maze(int w, int h) : g_width(w), g_height(h) {
            grid.createGrid(g_width, g_height);
        }

        // Create maze matrix
        void createMaze(){
            maze.resize(g_height, std::vector<Node*>(g_width));
            for (int y = 0; y < g_height; ++y) {
                for (int x = 0; x < g_width; ++x) {
                    Node* node = new Node(x, y);
                    maze[y][x] = node;
                }
            }

            fillMaze();
        }

        // Virtual maze fill
        virtual void fillMaze(){}
        
        // Return a random starting node
        Node* startingNode(){
            std::uniform_int_distribution<> randomWidth(0, grid.getWidth()-1);
            std::uniform_int_distribution<> randomHeight(0, grid.getHeight()-1);

            return grid.graph_matrix[randomHeight(rng)][randomWidth(rng)];
        }

        // Return the maze
        std::vector<std::vector<Node*>> getMaze() { return maze; }

        // Print the corresponding .world file
        void printGazebo(std::string filename){
            std::string path = get_current_dir() + "/../../world/";
            std::ofstream worldfile(path+filename+".world");
            
            // Verify that the worldfile exists
            if (!worldfile) {
                std::cerr << "Error opening " << filename << ".world for writing.\n";
                std::cerr << "Terminating.";
                exit(1);
            }

            float xmin = 0;
            float ymin = 0;
            float xmax = grid.getWidth();
            float ymax = grid.getHeight();
            int xresolution = (xmax - xmin + 2) * 30,
                yresolution = (ymax - ymin + 2) * 30;

            // svg parameters
            worldfile <<"<sdf version=\'1.6\'>\n" <<
                        "<world name=\'default\'>\n" <<
                        "<light name=\'sun\' type=\'directional\'>\n" <<
                        "<cast_shadows>1</cast_shadows>\n" <<
                        "<pose frame=\'\'>0 0 10 0 -0 0</pose>\n" <<
                        "<diffuse>0.8 0.8 0.8 1</diffuse>\n" <<
                        "<specular>0.2 0.2 0.2 1</specular>\n" <<
                        "<attenuation>\n" <<
                        "<range>1000</range>\n" <<
                        "<constant>0.9</constant>\n" <<
                        "<linear>0.01</linear>\n" <<
                        "<quadratic>0.001</quadratic>\n" <<
                        "</attenuation>\n" <<
                        "<direction>-0.5 0.1 -0.9</direction>\n" <<
                        "</light>\n" <<
                        "<model name=\'ground_plane\'>\n" <<
                        "<static>1</static>\n" <<
                        "<link name=\'link\'>\n" <<
                        "<collision name=\'collision\'>\n" <<
                        "<geometry>\n" <<
                        "<plane>\n" <<
                        "<normal>0 0 1</normal>\n" <<
                        "<size>100 100</size>\n" <<
                        "</plane>\n" <<
                        "</geometry>\n" <<
                        "<surface>\n" <<
                        "<friction>\n" <<
                        "<ode>\n" <<
                        "<mu>100</mu>\n" <<
                        "<mu2>50</mu2>\n" <<
                        "</ode>\n" <<
                        "<torsional>\n" <<
                        "<ode/>\n" <<
                        "</torsional>\n" <<
                        "</friction>\n" <<
                        "<contact>\n" <<
                        "<ode/>\n" <<
                        "</contact>\n" <<
                        "<bounce/>\n" <<
                        "</surface>\n" <<
                        "<max_contacts>10</max_contacts>\n" <<
                        "</collision>\n" <<
                        "<visual name=\'visual\'>\n" <<
                        "<cast_shadows>0</cast_shadows>\n" <<
                        "<geometry>\n" <<
                        "<plane>\n" <<
                        "<normal>0 0 1</normal>\n" <<
                        "<size>100 100</size>\n" <<
                        "</plane>\n" <<
                        "</geometry>\n" <<
                        "<material>\n" <<
                        "<script>\n" <<
                        "<uri>file://media/materials/scripts/gazebo.material</uri>\n" <<
                        "<name>Gazebo/Grey</name>\n" <<
                        "</script>\n" <<
                        "</material>\n" <<
                        "</visual>\n" <<
                        "<self_collide>0</self_collide>\n" <<
                        "<enable_wind>0</enable_wind>\n" <<
                        "<kinematic>0</kinematic>\n" <<
                        "</link>\n" <<
                        "</model>\n" <<
                        "<gravity>0 0 -9.8</gravity>\n" <<
                        "<magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>\n" <<
                        "<atmosphere type=\'adiabatic\'/>\n" <<
                        "<physics name=\'default_physics\' default=\'0\' type=\'ode\'>\n" <<
                        "<max_step_size>0.001</max_step_size>\n" <<
                        "<real_time_factor>1</real_time_factor>\n" <<
                        "<real_time_update_rate>1000</real_time_update_rate>\n" <<
                        "</physics>\n" <<
                        "<scene>\n" <<
                        "<ambient>0.4 0.4 0.4 1</ambient>\n" <<
                        "<background>0.7 0.7 0.7 1</background>\n" <<
                        "<shadows>1</shadows>\n" <<
                        "</scene>\n" <<
                        "<audio>\n" <<
                        "<device>default</device>\n" <<
                        "</audio>\n" <<
                        "<wind/>\n" <<
                        "<spherical_coordinates>\n" <<
                        "<surface_model>EARTH_WGS84</surface_model>\n" <<
                        "<latitude_deg>0</latitude_deg>\n" <<
                        "<longitude_deg>0</longitude_deg>\n" <<
                        "<elevation>0</elevation>\n" <<
                        "<heading_deg>0</heading_deg>\n" <<
                        "</spherical_coordinates>\n";

            // Create maze structure
            int id = 0;
            std::vector<Wall> wall_list = {};
            constexpr float half_pi = 1.570796;
            constexpr float wall_size = 1.8;
            for (int y = 0; y < grid.getHeight(); ++y) {
                for (int x = 0; x < grid.getWidth(); ++x) {
                    // Spawn bottom wall
                    spawn_wall(wall_list, id, round(((*maze[y][x]).posx*wall_size + (*maze[y][x]).posx*wall_size+wall_size)/2), round(((*maze[y][x]).posy*wall_size + (*maze[y][x]).posy*wall_size)/2), 0.0);
                    
                    // Spawn right wall
                    spawn_wall(wall_list, id, round(((*maze[y][x]).posx*wall_size+wall_size + (*maze[y][x]).posx*wall_size+wall_size)/2), round(((*maze[y][x]).posy*wall_size + (*maze[y][x]).posy*wall_size+wall_size)/2), half_pi);
                    
                    // Spawn top wall
                    if (y == grid.getHeight()-1)
                        spawn_wall(wall_list, id, round(((*maze[y][x]).posx*wall_size + (*maze[y][x]).posx*wall_size+wall_size)/2), round(((*maze[y][x]).posy*wall_size+wall_size + (*maze[y][x]).posy*wall_size+wall_size)/2), 0.0);
                    
                    // Spawn left wall
                    if (x == 0)
                        spawn_wall(wall_list, id, round(((*maze[y][x]).posx*wall_size + (*maze[y][x]).posx*wall_size)/2), round(((*maze[y][x]).posy*wall_size + (*maze[y][x]).posy*wall_size+wall_size)/2), half_pi);
                }
            }

            // Remove path walls
            for (int y = 0; y < grid.getHeight(); ++y) {
                for (int x = 0; x < grid.getWidth(); ++x) {
                    //std::cout<<"node ["<< (*maze[y][x]).posx << ", "<<(*maze[y][x]).posy<<"] connected to -> ";
                    for(int e = 0; e < (*maze[y][x]).edges.size(); ++e) {
                        //std::cout<<"["<<(*maze[y][x]).edges[e]->posx << ", "<<(*maze[y][x]).edges[e]->posy<<"] -> ";
                        int ex = (*maze[y][x]).edges[e]->posx,
                            ey = (*maze[y][x]).edges[e]->posy;
                        int nx = (*maze[y][x]).posx,
                            ny = (*maze[y][x]).posy;
                        float x1 = nx,
                              x2 = nx;
                        float y1 = ny,
                              y2 = ny;
                    
                        if (ex > nx) { x1 = nx + 1; x2 = nx + 1; }
                        else if (ex  == nx) { x1 = nx; x2 = nx + 1; }
                        else if (ex  < nx) { x1 = nx; x2 = nx; }
                        
                        if (ey > ny) { y1 = ny + 1; y2 = ny + 1; }
                        else if (ey == ny) { y1 = ny; y2 = ny + 1; }
                        else if (ey < ny) { y1 = ny; y2 = ny; }
                        //std::cout << "X1: " << (x1+x2)/2 << ", X2: " << (y1+y2)/2 << std::endl;
                        remove_wall(wall_list, round(wall_size*(x1+x2)/2), round(wall_size*(y1+y2)/2));
                    }
                    //std::cout << std::endl;
                }
            }
            
            for (auto it = wall_list.begin(); it != wall_list.end(); ++it)
                worldfile << it->print_sdf();
            
            worldfile <<"<state world_name=\'default\'>\n" <<
                        "<sim_time>33 531000000</sim_time>\n" <<
                        "<real_time>33 696388605</real_time>\n" <<
                        "<wall_time>1601243306 962712131</wall_time>\n" <<
                        "<iterations>33531</iterations>\n" <<
                        "<model name=\'ground_plane\'\n>" <<
                        "<pose frame=\'\'>0 0 0 0 -0 0</pose>\n" <<
                        "<scale>1 1 1</scale>\n" <<
                        "<link name=\'link\'>\n" <<
                        "<pose frame=\'\'>0 0 0 0 -0 0</pose>\n" <<
                        "<velocity>0 0 0 0 -0 0</velocity>\n" <<
                        "<acceleration>0 0 0 0 -0 0</acceleration>\n" <<
                        "<wrench>0 0 0 0 -0 0</wrench>\n" <<
                        "</link>\n" <<
                        "</model>\n" <<
                        "<light name=\'sun\'>\n" <<
                        "<pose frame=\'\'>0 0 10 0 -0 0</pose>\n" <<
                        "</light>\n" <<
                        "</state>\n" <<
                        "<gui fullscreen=\'0\'>\n" <<
                        "<camera name=\'user_camera\'>\n" <<
                        "<pose frame=\'\'>5.94249 -3.74514 2.36968 -0 0.383643 2.48819</pose>\n" <<
                        "<view_controller>orbit</view_controller>\n" <<
                        "<projection_type>perspective</projection_type>\n" <<
                        "</camera>\n" <<
                        "</gui>\n" <<
                        "</world>\n" <<
                        "</sdf>";

            std::cout << "A new maze was generated" << std::endl;
        }

        struct Wall {
            int id;
            float posx, posy, orientation;

            std::string print_sdf() {
                std::stringstream wallsdf;
                wallsdf << "<model name=\"labyrinth_wall" << std::to_string(this->id) << "\">\n" <<
                "<pose>" << std::to_string(this->posx) << " " << std::to_string(this->posy) << 
                   " 0.9 0 0 " << std::to_string(this->orientation) << "</pose>\n" <<
                "<link name=\"link\">\n" <<
                "<inertial>\n" <<
                "<mass>3.0</mass>\n" <<
                "</inertial>\n" <<
                "<collision name=\"collision\">\n" <<
                "<geometry>\n" <<
                "<box>\n" <<
                "<size>1.8 0.5 1.8</size>\n" <<
                "</box>\n" <<
                "</geometry>\n" <<
                "</collision>\n" <<
                "<visual name=\"visual\">\n" <<
                "<geometry>\n" <<
                "<box>\n" <<
                "<size>1.8 0.5 1.8</size>\n" <<
                "</box>\n" <<
                "</geometry>\n" <<
                "</visual>\n" <<
                "</link>\n" <<
                "<static>true</static>\n" <<
                "</model>\n";

                return wallsdf.str();
            }

            Wall() = delete;
            Wall(int _id = 0, float _posx = 0, float _posy = 0, float _orientation = 0.0) : 
                id(_id), posx(_posx), posy(_posy), orientation(_orientation) {}
        };

        void spawn_wall(std::vector<Wall>& wall_list, int& id, const float& x, const float& y, const float& th) {
            Wall _wall = Wall(id, x, y, th);
            wall_list.emplace_back(_wall);
            id++;
        }

        void remove_wall(std::vector<Wall>& wall_list, const float& x, const float& y) {
            for (auto it = wall_list.begin(); it != wall_list.end(); it++) {
                if (it->posx == x && it->posy == y) {
                    wall_list.erase(it);
                    return;
                }
            }
        }

        float round(float var) {
            float value = (int)(var * 100 + .5);
            return (float)value / 100;
        }
};