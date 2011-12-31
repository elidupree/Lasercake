/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012
    
    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

   
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "SDL.h"
#include "GL/gl.h"
#include "GL/glu.h"

#include <iostream>

#include "world.hpp"

static SDL_Surface *gScreen;

static void initAttributes ()
{
    // Setup attributes we want for the OpenGL context
    
    int value;
    
    // Don't set color bit sizes (SDL_GL_RED_SIZE, etc)
    //    Mac OS X will always use 8-8-8-8 ARGB for 32-bit screens and
    //    5-5-5 RGB for 16-bit screens
    
    // Request a 16-bit depth buffer (without this, there is no depth buffer)
    value = 16;
    SDL_GL_SetAttribute (SDL_GL_DEPTH_SIZE, value);
    
    
    // Request double-buffered OpenGL
    //     The fact that windows are double-buffered on Mac OS X has no effect
    //     on OpenGL double buffering.
    value = 1;
    SDL_GL_SetAttribute (SDL_GL_DOUBLEBUFFER, value);
}

static void printAttributes ()
{
    // Print out attributes of the context we created
    int nAttr;
    int i;
    
    int  attr[] = { SDL_GL_RED_SIZE, SDL_GL_BLUE_SIZE, SDL_GL_GREEN_SIZE,
                    SDL_GL_ALPHA_SIZE, SDL_GL_BUFFER_SIZE, SDL_GL_DEPTH_SIZE };
                    
    const char *desc[] = { "Red size: %d bits\n", "Blue size: %d bits\n", "Green size: %d bits\n",
                     "Alpha size: %d bits\n", "Color buffer size: %d bits\n", 
                     "Depth bufer size: %d bits\n" };

    nAttr = sizeof(attr) / sizeof(int);
    
    for (i = 0; i < nAttr; i++) {
    
        int value;
        SDL_GL_GetAttribute ((SDL_GLattr)attr[i], &value);
        printf (desc[i], value);
    } 
}

static void createSurface (int fullscreen)
{
    Uint32 flags = 0;
    
    flags = SDL_OPENGL;
    if (fullscreen)
        flags |= SDL_FULLSCREEN;
    
    // Create window
    gScreen = SDL_SetVideoMode (640, 640, 0, flags);
    if (gScreen == NULL) {
		
        fprintf (stderr, "Couldn't set 640x640 OpenGL video mode: %s\n",
                 SDL_GetError());
		SDL_Quit();
		exit(2);
	}
}



struct world_building_func {
  world_building_func(std::string scenario):scenario(scenario){}
  std::string scenario;
  void operator()(world_building_gun make, tile_bounding_box bounds) {
    const tile_coordinate wc = world_center_coord;
    if (scenario.substr(0,15) == "pressure_tunnel") {
      for(vector3<tile_coordinate> l : bounds) {
        const tile_coordinate tower_lower_coord = wc;
        const tile_coordinate tower_upper_coord = wc+10;
        const tile_coordinate tower_height = 200;
        if (l.x < tower_lower_coord && l.y >= tower_lower_coord && l.y <= tower_lower_coord && l.z >= wc && l.z <= wc) {}
        else if (l.x < tower_lower_coord && l.y >= tower_lower_coord-1 && l.y <= tower_lower_coord+1 && l.z >= wc-1 && l.z <= wc+1)
          make(ROCK, l);
        else if (l.x >= tower_lower_coord && l.x < tower_upper_coord && l.y >= tower_lower_coord && l.y < tower_upper_coord && l.z >= wc && l.z < wc + tower_height)
          make(WATER, l);
        else if (l.x >= tower_lower_coord-1 && l.x < tower_upper_coord+1 && l.y >= tower_lower_coord-1 && l.y < tower_upper_coord+1 && l.z >= wc-1 && l.z < wc + tower_height+1)
          make(ROCK, l);
      }
      return;
    }
    for (tile_coordinate x = std::max(world_center_coord-1, bounds.min.x); x < std::min(world_center_coord+21, bounds.min.x + bounds.size.x); ++x) {
      for (tile_coordinate y = std::max(world_center_coord-1, bounds.min.y); y < std::min(world_center_coord+21, bounds.min.y + bounds.size.y); ++y) {
        for (tile_coordinate z = std::max(world_center_coord-1, bounds.min.z); z < std::min(world_center_coord+21, bounds.min.z + bounds.size.z); ++z) {
          vector3<tile_coordinate> l(x,y,z);
          if (x == world_center_coord-1 || x == world_center_coord+20 || y == world_center_coord-1 || y == world_center_coord+20 || z == world_center_coord-1 || z == world_center_coord+20) {
            make(ROCK, l);
          }
          else {
            if ((scenario.substr(0,5) == "tower") &&
                  x >= wc+4 && x <= wc+6 &&
                  y >= wc+4 && y <= wc+6 &&
                  z >= wc+1) make(WATER, l);
            else if ((scenario == "tower2" || scenario == "tower3") &&
                  x >= wc+3 && x <= wc+7 &&
                  y >= wc+3 && y <= wc+7 &&
                  z >= wc+1) make(ROCK, l);
            else if (scenario == "tower3" && z < wc+3 && (
                  x == wc+1 || x == wc+9 || y == wc+1 || wc+y == 9)) make(ROCK, l);
                  
            if (scenario == "shallow") {
                   if (z == wc+0 && x >= wc+5) make(ROCK, l);
              else if (z == wc+1 && x >= wc+10) make(ROCK, l);
              else if (z == wc+2 && x >= wc+15) make(ROCK, l);
              else if (x == wc+19) make(WATER, l);
            }
            if (scenario == "steep") {
              if (z < 20 - x) make(ROCK, l);
              else if (z >= wc+15 && (wc + 20 - x) >= 15) make(WATER, l);
            }
            if (scenario == "tank") {
              if (z < wc+8 && x > wc+10) make(ROCK, l);
              else if (x >= wc+12 && y <= wc+13 && y >= wc+7) make(WATER, l);
              else if (y != wc+10 && x > wc+10) make(ROCK, l);
              else if (z > wc+8 && x > wc+10 && x < wc+18) make(ROCK, l);
              else if (x > wc+10) make(WATER, l);
            }
            if (scenario == "tank2") {
              if (z < wc+8 && x > wc+10) make(ROCK, l);
              else if (x >= wc+12 && y <= wc+13 && y >= wc+7) make(WATER, l);
              else if (y != wc+9 && y != wc+10 && x > wc+10) make(ROCK, l);
              else if (z > wc+9 && x > wc+10 && x < wc+18) make(ROCK, l);
              else if (x > wc+10) make(WATER, l);
            }
            if (scenario == "twisty") {
              if (x == wc+0) make(WATER, l);
              else if (x == wc+1 && z > wc+0) make(ROCK, l);
              else if (x == wc+5) make(ROCK, l);
              else if (x == wc+2 && (z % 4) == 1) make(ROCK, l);
              else if (x == wc+3 && (z % 2) == 1) make(ROCK, l);
              else if (x == wc+4 && (z % 4) == 3) make(ROCK, l);
            }
          }
        }
      }
    }
  }
};

struct vertex_entry { GLfloat x, y, z; };
void push_vertex(vector<vertex_entry> &v, GLfloat x, GLfloat y, GLfloat z) {
  v.push_back((vertex_entry){x,y,z});
}

static void mainLoop (std::string scenario)
{
  SDL_Event event;
  int done = 0;
  int frame = 0;
  int p_mode = 0;
srand(time(NULL));

  world w{world::worldgen_function_t(world_building_func(scenario))};
  shared_ptr<laser_emitter> foo (new laser_emitter(convert_to_high_resolution(world_center_coords) + vector3<high_resolution_coordinate>(10ULL << 32, 10ULL << 32, 10ULL << 32), vector3<high_resolution_delta>(5,3,2)));
  w.queue_creating_mobile_object(foo);
  
  double view_x = 5, view_y = 5, view_z = 5, view_dist = 20;
    
  while ( !done ) {

    /* Check for events */
    while ( SDL_PollEvent (&event) ) {
      switch (event.type) {
        case SDL_MOUSEMOTION:
          break;
          
        case SDL_MOUSEBUTTONDOWN:
          break;
          
        case SDL_KEYDOWN:
          if(event.key.keysym.sym == SDLK_p) ++p_mode;
          if(event.key.keysym.sym == SDLK_q) ++view_x;
          if(event.key.keysym.sym == SDLK_a) --view_x;
          if(event.key.keysym.sym == SDLK_w) ++view_y;
          if(event.key.keysym.sym == SDLK_s) --view_y;
          if(event.key.keysym.sym == SDLK_e) ++view_z;
          if(event.key.keysym.sym == SDLK_d) --view_z;
          if(event.key.keysym.sym == SDLK_r) ++view_dist;
          if(event.key.keysym.sym == SDLK_f) --view_dist;
          if(event.key.keysym.sym != SDLK_ESCAPE)break;
          
        case SDL_QUIT:
          done = 1;
          break;
          
        default:
          break;
      }
    }
	
    if(p_mode == 1)continue;
    if(p_mode > 1)--p_mode;
    int before_drawing = SDL_GetTicks();

    //drawing code here
    vector<vertex_entry> rock_vertices;
    vector<vertex_entry> sticky_water_vertices;
    vector<vertex_entry> free_water_vertices;
    vector<vertex_entry> velocity_vertices;
    vector<vertex_entry> progress_vertices;
    //vector<vertex_entry> inactive_marker_vertices;
    vector<vertex_entry> laserbeam_vertices;
    
    unordered_set<object_identifier> tiles_to_draw;
    w.collect_things_exposed_to_collision_intersecting(tiles_to_draw, tile_bounding_box(vector3<tile_coordinate>(world_center_coord + view_x - 50, world_center_coord + view_y - 50, world_center_coord + view_z - 50), vector3<tile_coordinate>(101,101,101)));
    
    for (auto p : w.laser_sfxes) {
      vector3<double> locv = vector3<double>(vector3<high_resolution_delta>(p.first - convert_to_high_resolution(world_center_coords))) / double(1ULL << 32);
      vector3<double> locv2 = vector3<double>(vector3<high_resolution_delta>(p.first + p.second - convert_to_high_resolution(world_center_coords))) / double(1ULL << 32);
      //std::cerr << locv.x << " !l " << locv.y << " !l " << locv.z << "\n";
      //std::cerr << locv2.x << " !l " << locv2.y << " !l " << locv2.z << "\n";
      push_vertex(laserbeam_vertices, locv.x, locv.y, locv.z-0.5);
      push_vertex(laserbeam_vertices, locv.x, locv.y, locv.z+0.5);
      push_vertex(laserbeam_vertices, locv.x, locv.y, locv.z);
      GLfloat length = (locv2 - locv).magnitude();
      for (int i = 0; i < length; ++i) {
        vector3<double> locv3 = (locv + (((locv2 - locv) * i) / length));
        push_vertex(laserbeam_vertices, locv3.x, locv3.y, locv3.z);
        push_vertex(laserbeam_vertices, locv3.x, locv3.y, locv3.z);
      }
      push_vertex(laserbeam_vertices, locv2.x, locv2.y, locv2.z);
    }
    for (object_identifier const& id : tiles_to_draw) {
      /*if (mobile_object_identifier const* mid = id.get_mobile_object_identifier()) {
        if (shared_ptr<mobile_object> *objp = w.get_mobile_object(loc)) {
          (*objp)->
        }
      }*/
     if (tile_location const* locp = id.get_tile_location()) {
      tile_location const& loc = *locp;
      tile const& t = loc.stuff_at();
      vector3<GLfloat> locv(vector3<tile_coordinate_signed_type>(loc.coords() - world_center_coords)); // TODO : properly speaking, "minus the perspective you're looking from"? world_center_coords has no business in any code except the world generation, I think
      
      // Hack - TODO remove
      if (frame == 0 && t.contents() == WATER) w.activate_water(loc);
      
      vector<vertex_entry> *vect;
      
           if(t.contents() == ROCK) vect = &        rock_vertices;
      else if(t.is_sticky_water() ) vect = &sticky_water_vertices;
      else if(t.is_free_water  () ) vect = &  free_water_vertices;
      else assert(false);
      
      push_vertex(*vect, locv.x,     locv.y,     locv.z + 0.5);
      push_vertex(*vect, locv.x + 1, locv.y,     locv.z + 0.5);
      push_vertex(*vect, locv.x + 1, locv.y + 1, locv.z + 0.5);
      push_vertex(*vect, locv.x,     locv.y + 1, locv.z + 0.5);
      
      if (t.contents() == WATER) {
        if (water_movement_info *water = w.get_active_water_tile(loc)) {
          push_vertex(velocity_vertices, locv.x+0.5, locv.y+0.5, locv.z + 0.5);
          push_vertex(velocity_vertices,
              locv.x + 0.5 + ((GLfloat)water->velocity.x / (250 * precision_factor)),
              locv.y + 0.5 + ((GLfloat)water->velocity.y / (250 * precision_factor)),
              locv.z + 0.5 + ((GLfloat)water->velocity.z / (250 * precision_factor)));
          
          for (EACH_CARDINAL_DIRECTION(dir)) {
            const sub_tile_distance prog = water->progress[dir];
            if (prog > 0) {
              vector3<GLfloat> directed_prog = (vector3<GLfloat>(dir.v) * prog) / progress_necessary;

              push_vertex(progress_vertices, locv.x + 0.5, locv.y + 0.5, locv.z + 0.5);
              push_vertex(progress_vertices,
                  locv.x + 0.5 + directed_prog.x,
                  locv.y + 0.5 + directed_prog.y,
                  locv.z + 0.5 + directed_prog.z);
            }
          }
        }
        /*else {
          push_vertex(inactive_marker_vertices, locv.x + 0.5, locv.y + 0.5, locv.z + 0.5);
        }*/
      }
     }
    }
    
    int before_GL = SDL_GetTicks();

    glClear(GL_COLOR_BUFFER_BIT);
    frame += 1;
    glLoadIdentity();
    gluPerspective(80, 1, 1, 100);
    gluLookAt(view_x + view_dist * std::cos((double)frame / 40.0),view_y + view_dist * std::sin((double)frame / 40.0),view_z + (view_dist / 2) + (view_dist / 4) * std::sin((double)frame / 60.0),view_x,view_y,view_z,0,0,1);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    
    /*glColor4f(0.2,0.4,0.0,1.0);
    glVertexPointer(3, GL_FLOAT, 0, &ground_vertices[0]);
    glDrawArrays(GL_QUADS, 0, ground_vertices.size());*/
    
    glColor4f(0.5,0.0,0.0,0.5);
    glVertexPointer(3, GL_FLOAT, 0, &rock_vertices[0]);
    glDrawArrays(GL_QUADS, 0, rock_vertices.size());
    
    glColor4f(0.0, 0.0, 1.0, 0.5);
    glVertexPointer(3, GL_FLOAT, 0, &sticky_water_vertices[0]);
    glDrawArrays(GL_QUADS, 0, sticky_water_vertices.size());
    
    glColor4f(0.4, 0.4, 1.0, 0.5);
    glVertexPointer(3, GL_FLOAT, 0, &free_water_vertices[0]);
    glDrawArrays(GL_QUADS, 0, free_water_vertices.size());
    
    glColor4f(0.0, 1.0, 0.0, 0.5);
    glVertexPointer(3, GL_FLOAT, 0, &velocity_vertices[0]);
    glDrawArrays(GL_LINES, 0, velocity_vertices.size());
    
    glColor4f(0.0, 0.0, 1.0, 0.5);
    glVertexPointer(3, GL_FLOAT, 0, &progress_vertices[0]);
    glDrawArrays(GL_LINES, 0, progress_vertices.size());
    
    /*glColor4f(0.0, 0.0, 0.0, 0.5);
    glPointSize(3);
    glVertexPointer(3, GL_FLOAT, 0, &inactive_marker_vertices[0]);
    glDrawArrays(GL_POINTS, 0, inactive_marker_vertices.size());*/
    
    glColor4f(0.0, 1.0, 0.0, 0.5);
    glLineWidth(3);
    glVertexPointer(3, GL_FLOAT, 0, &laserbeam_vertices[0]);
    glDrawArrays(GL_LINES, 0, laserbeam_vertices.size());
    
    
    
    glFinish();	
    SDL_GL_SwapBuffers();
   
    int before_processing = SDL_GetTicks();
    
    //doing stuff code here
    update(w);
    
    int after = SDL_GetTicks();
    std::cerr << (after - before_processing) << ", " << (before_GL - before_drawing) << ", " << (before_processing - before_GL) << "\n";

//    SDL_Delay(50);
  }
}

int main(int argc, char *argv[])
{
	// Init SDL video subsystem
	if ( SDL_Init (SDL_INIT_VIDEO) < 0 ) {
		
        fprintf(stderr, "Couldn't initialize SDL: %s\n",
			SDL_GetError());
		exit(1);
	}

    // Set GL context attributes
    initAttributes ();
    
    // Create GL context
    createSurface (0);
    
    // Get GL context attributes
    printAttributes ();
    
    // Init GL state
	gluPerspective(90, 1, 1, 100);
	gluLookAt(20,20,20,0,0,0,0,0,1);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    
    // Draw, get events...
    if (argc < 2) {
      std::cerr << "You didn't give an argument saying which scenario to use! Using default value...";
      mainLoop ("default");
    }
    else mainLoop (argv[1]);
    
    // Cleanup
	SDL_Quit();
	
    return 0;
}
