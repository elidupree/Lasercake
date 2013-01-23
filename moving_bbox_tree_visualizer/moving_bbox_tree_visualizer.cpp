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
#include <cmath>

#include <array>
#include <vector>

#include "../utils.hpp"
//#include "../data_structures/polygon_collision_detection.hpp"

const size_t MIN_NICE_NODE_SIZE = 4;

typedef int object_id;
typedef lasercake_int<int64_t>::type time_int_type;
typedef non_normalized_rational<time_int_type> time_type;
//typedef faux_optional<time_type> optional_time;
typedef boost::random::uniform_int_distribution<int64_t> uniform_random;

template <int NumDimensions>
struct bbox {
  std::array<int64_t, NumDimensions> min;
  std::array<int64_t, NumDimensions> max;

  bbox combined_with(bbox const& o)const {
    bbox result;
    for (int dim = 0; dim < NumDimensions; ++dim) {
      result.max[dim] = std::max(max[dim], o.max[dim]);
      result.min[dim] = std::min(min[dim], o.min[dim]);
    }
    return result;
  }
  int64_t size(int dim)const { return max[dim] - min[dim]; }
  int64_t maxdim()const {
    int64_t result = 0;
    for (int dim = 0; dim < NumDimensions; ++dim) {
      if (size(dim) > result) result = size(dim);
    }
    return result;
  }

  bool fitsnicely(bbox const& o)const {
    /*for (int dim = 0; dim < NumDimensions; ++dim) {
      if ((size(dim) << 1) > o.size(dim)) {
        return false;
      }
    }
    return true;*/
    return (maxdim() << 1) <= o.maxdim();
  }
};

template <int NumDimensions>
struct moving_object {
  object_id id;
  bbox<NumDimensions> phys_bounds;
  std::array<int64_t, NumDimensions> vel;

  bbox<NumDimensions*2> bounds()const {
    bbox<NumDimensions*2> result;
    for (int dim = 0; dim < NumDimensions; ++dim) {
      result.max[dim] = phys_bounds.max[dim];
      result.min[dim] = phys_bounds.min[dim];
      result.max[dim+NumDimensions] = vel[dim];
      result.min[dim+NumDimensions] = vel[dim];
    }
    return result;
  }
};

template <int NumDimensions>
struct tree_node {
  bbox<NumDimensions*2> bounds;
  std::vector<tree_node> children;
  std::vector<moving_object<NumDimensions>> stuff_here;
  
  int64_t strain(moving_object<NumDimensions> const& o)const {
    bbox<NumDimensions*2> combined = bounds.combined_with(o.bounds());
    int64_t result = 100 * (combined.maxdim() - bounds.maxdim());
    for (int dim = 0; dim < NumDimensions*2; ++dim) result += combined.size(dim) - bounds.size(dim);
    return result;
  }

  void insert(moving_object<NumDimensions> const& o) {
    //    std::cerr << "Insert attempt:\n";
    bounds = bounds.combined_with(o.bounds());
    if (!o.bounds().fitsnicely(bounds)) {
      stuff_here.push_back(o);
      //  std::cerr << "BZX\n";
      return;
    }
    int best_child = -1;
    int64_t least_strain = -1;
    for (size_t i = 0; i < children.size(); ++i) {
      int64_t s = children[i].strain(o);
      if (best_child == -1 || s < least_strain) {
        best_child = i;
        least_strain = s;
      }
    }
    if (best_child != -1) {
      //std::cerr << least_strain << "\n";
      tree_node& bc = children[best_child];
      if (bc.bounds.combined_with(o.bounds()).fitsnicely(bounds)) {
        //std::cerr << "foo\n";
        bc.insert(o);
        return;
      }
    }
    //else {
      std::vector<int> pulled_stuffs;
      tree_node child_attempt;
      child_attempt.stuff_here.push_back(o);
      child_attempt.bounds = o.bounds();
      for (size_t i = 0; i < stuff_here.size(); ++i) {
        if (child_attempt.bounds.combined_with(stuff_here[i].bounds()).fitsnicely(bounds)) {
          child_attempt.bounds = child_attempt.bounds.combined_with(stuff_here[i].bounds());
          pulled_stuffs.push_back(i);
        }
      }
      if (pulled_stuffs.size() >= MIN_NICE_NODE_SIZE) {
        for (int i = pulled_stuffs.size() - 1; i >= 0; --i) {
          child_attempt.stuff_here.push_back(stuff_here[pulled_stuffs[i]]);
          stuff_here[pulled_stuffs[i]] = stuff_here.back(); stuff_here.pop_back();
        }
        //std::cerr << "PULL! Woot!\n";
        children.push_back(child_attempt);
      }
      else {
        //std::cerr << "plunk\n";
        stuff_here.push_back(o);
      }
    //}
  }

  void search(std::vector<object_id>& results,
        moving_object<NumDimensions> const& o,
        time_type start_time,
        time_type end_time)const
  {
    time_type first_possible_overlap = start_time;
    time_type last_possible_overlap = end_time;
    for (int dim = 0; dim < NumDimensions; ++dim) {
      // basically
      // relative velocity = relative position / time
      // time = relative position / relative velocity
      // time = (bounds.something[dim] - o.phys_bounds.something[dim]) / (bounds.something[dim + NUM_DIMENSIONS] - o.velocity[dim])
      // Three of those have multiple possibilities. We're looking for the min and max time.
      // If the denominator is always positive or always negative, it's easy to compute min/max times.
      // If it crosses zero, then there's no min or max.
      const int64_t max_num = (bounds.max[dim] - o.phys_bounds.min[dim]);
      const int64_t min_num = (bounds.min[dim] - o.phys_bounds.max[dim]);
      const int64_t max_denom = (bounds.max[dim + NumDimensions] - o.velocity[dim]);
      const int64_t min_denom = (bounds.min[dim + NumDimensions] - o.velocity[dim]);
           if (min_denom > 0) {
        const time_type max_time(max_num, min_denom);
        const time_type min_time(min_num, max_denom);
        assert(min_time < max_time);
        if (max_time <  last_possible_overlap)  last_possible_overlap = max_time; // Duplicate code!!!
        if (min_time > first_possible_overlap) first_possible_overlap = min_time; // Duplicate code!!!
      }
      else if (max_denom < 0) {
        const time_type max_time(min_num, max_denom);
        const time_type min_time(max_num, min_denom);
        assert(min_time < max_time);
        if (max_time <  last_possible_overlap)  last_possible_overlap = max_time; // Duplicate code!!!
        if (min_time > first_possible_overlap) first_possible_overlap = min_time; // Duplicate code!!!
      }
      else if (min_denom == 0) {
        if (max_denom == 0) {
          if ((min_num > 0) || (max_num < 0)) {
            // They are moving at the same speed and do not overlap.
            return;
          }
          // else { They are moving at the same speed so they overlap forever. Restrict nothing. }
        }
        else if (min_num > 0) {
          // Arbitrarily high, but has a min
          const time_type min_time(min_num, max_denom);
          if (min_time > first_possible_overlap) first_possible_overlap = min_time; // Duplicate code!!!
        }
        else if (max_num < 0) {
          // Arbitrarily low, but has a max
          const time_type max_time(max_num, max_denom);
          if (max_time <  last_possible_overlap)  last_possible_overlap = max_time; // Duplicate code!!!
        }
      }
      else if (max_denom == 0) {
        assert(min_denom != 0);
        if (min_num > 0) {
          // Arbitrarily low, but has a max
          const time_type max_time(min_num, min_denom);
          if (max_time <  last_possible_overlap)  last_possible_overlap = max_time; // Duplicate code!!!
        }
        else if (max_num < 0) {
          // Arbitrarily high, but has a min
          const time_type min_time(max_num, min_denom);
          if (min_time > first_possible_overlap) first_possible_overlap = min_time; // Duplicate code!!!
        }
      }
      else {
        // This is what we call a "split node"; it could be going in either direction,
        // and so its possible overlaps are not an interval, but the complement of an interval
        // (or, in the case where its x values also overlap, its possible overlaps are every time.)

        if (min_num > 0) {
          const time_type    exit_time(min_num, min_denom);
          const time_type reentry_time(max_num, max_denom);
          assert(exit_time < reentry_time);

          // Duplicate code!!!
               if (reentry_time >  last_possible_overlap) { if (   exit_time <  last_possible_overlap) {  last_possible_overlap =    exit_time; } }
          else if (   exit_time < first_possible_overlap) { if (reentry_time > first_possible_overlap) { first_possible_overlap = reentry_time; } }
          else {
            // In this situation, the proper resulting shape would be not one interval, but TWO intervals.
            // To keep it simple, we just leave it as a single contiguous interval.
          }
        }
        else if (max_num < 0) {
          const time_type    exit_time(min_num, max_denom);
          const time_type reentry_time(max_num, min_denom);
          assert(exit_time < reentry_time);

          // Duplicate code!!!
               if (reentry_time >  last_possible_overlap) { if (   exit_time <  last_possible_overlap) {  last_possible_overlap =    exit_time; } }
          else if (   exit_time < first_possible_overlap) { if (reentry_time > first_possible_overlap) { first_possible_overlap = reentry_time; } }
          else {
            // In this situation, the proper resulting shape would be not one interval, but TWO intervals.
            // To keep it simple, we just leave it as a single contiguous interval.
          }
        }
        // else { The boxes overlap and could be going at relative speed zero, so they could overlap at any time, so restrict nothing }
      }
    }

    if (first_possible_overlap <= last_possible_overlap) {
      // The object's trajectory overlaps some part of this box. Check our stuff-here and children.
      
      for (auto const& o2 : stuff_here) {
        time_type first_collision_moment = first_possible_overlap;
        time_type  last_collision_moment =  last_possible_overlap;
        for (int dim = 0; dim < NumDimensions; ++dim) {
          const int64_t max_num = (o2.phys_bounds.max[dim] - o.phys_bounds.min[dim]);
          const int64_t min_num = (o2.phys_bounds.min[dim] - o.phys_bounds.max[dim]);
          const int64_t denom = (o2.velocity[dim + NumDimensions] - o.velocity[dim]);
          if (denom == 0) {
            if ((max_num > 0) || (min_num < 0)) {
              // Same speed, never overlap - hack : never colliding
              first_collision_moment = time_type(1);
               last_collision_moment = time_type(0);
            }
            // else { They are moving at the same speed so they overlap forever. Restrict nothing. }
          }
          else {
            const time_type min_time(denom > 0 ? min_num : max_num, denom);
            const time_type max_time(denom > 0 ? min_num : max_num, denom);
            if (max_time <  last_collision_moment)  last_collision_moment = max_time; // Duplicate code!!!
            if (min_time > first_collision_moment) first_collision_moment = min_time; // Duplicate code!!!
          }
        }
        if (first_collision_moment <= last_collision_moment) {
          // Note - not returning the time of collision
          results.insert(o2.object_id);
        }
      }

      for (auto const& c : children) c.search(results, start_time, end_time);
    }
  }
};






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


void draw_4d_point(int64_t x, int64_t y, int64_t vx, int64_t vy) {
  glColor3f(1.0, 0.5 + ((GLfloat)vx) * 0.1, 0.5 + ((GLfloat)vy) * 0.1);
  glVertex3f((GLfloat)x + (GLfloat)vx / 2.0, (GLfloat)y + (GLfloat)vy / 2.0, ((GLfloat)(vx + vy)) * 433.0 / 500.0);
}

void draw_4d_bbox(bbox<4> const& b) {
  glBegin(GL_LINES);
  for (int i = 0; i < (1 << 4); ++i) {
    for (int j = 0; j < 4; ++j) {
      if (!(i & (1 << j))) {
        draw_4d_point(
          (i & (1 << 0)) ? b.min[0] : b.max[0],
          (i & (1 << 1)) ? b.min[1] : b.max[1],
          (i & (1 << 2)) ? b.min[2] : b.max[2],
          (i & (1 << 3)) ? b.min[3] : b.max[3]);
        draw_4d_point(
          ((i & (1 << 0)) || (j == 0)) ? b.min[0] : b.max[0],
          ((i & (1 << 1)) || (j == 1)) ? b.min[1] : b.max[1],
          ((i & (1 << 2)) || (j == 2)) ? b.min[2] : b.max[2],
          ((i & (1 << 3)) || (j == 3)) ? b.min[3] : b.max[3]);
      }
    }
  }
  glEnd();
}

void draw_node(tree_node<2> const& node) {
  draw_4d_bbox(node.bounds);
  for (auto const& c : node.children) {
    draw_node(c);
  }
  for (auto const& c : node.stuff_here) {
    draw_4d_bbox(c.bounds());
  }
}

void nodecount_search(std::vector<int>& counts, std::vector<int>& objcounts, tree_node<2> const& node, size_t level) {
  while (counts.size() <= level) counts.push_back(0);
  while (objcounts.size() <= level) objcounts.push_back(0);
  ++counts[level];
  objcounts[level] += node.stuff_here.size();
  for (auto const& c : node.children) nodecount_search(counts, objcounts, c, level + 1);
}

void print_nodecount(tree_node<2> const& root) {
  std::cerr << "\n=========== NODECOUNT =============\n";
  std::vector<int> counts;
  std::vector<int> objcounts;
  nodecount_search(counts, objcounts, root, 0);
  for (size_t level = 0; level < counts.size(); ++level) {
    std::cerr << "Nodes at level " << level << ": " << counts[level] << "\n";
    std::cerr << "Objects stored at level " << level << ": " << objcounts[level] << "\n";
  }
}

void do_2d_test_scenario(tree_node<2>& root) {
  object_id next_id = 0;
  root.children.clear();
  root.stuff_here.clear();
  for (int dim = 0; dim < 4; ++dim) {
    root.bounds.min[dim] = 0;
    root.bounds.max[dim] = 1;
  }
  large_fast_noncrypto_rng rng(0);
  uniform_random rand_size(1000,3000);

  // A bunch of clumps.
  for (int i = 0; i < 100; ++i) {
    uniform_random rand_loc(-1000000,1000000);
    uniform_random rand_square_root_num_objects_times_100(40,300);
    int64_t xloc = rand_loc(rng);
    int64_t yloc = rand_loc(rng);
    int64_t square_root_num_objects_times_100 = rand_square_root_num_objects_times_100(rng);
    for (int j = 0; j < (square_root_num_objects_times_100 * square_root_num_objects_times_100) / 100; ++j) {
      uniform_random rand_obj_loc(-square_root_num_objects_times_100 * 100, square_root_num_objects_times_100 * 100);
      uniform_random rand_vel(-100,100);

      int64_t xloc_offs = rand_obj_loc(rng);
      int64_t yloc_offs = rand_obj_loc(rng);
      moving_object<2> o;
      o.id = next_id++;
      o.phys_bounds.min[0] = xloc + xloc_offs;
      o.phys_bounds.max[0] = o.phys_bounds.min[0] + rand_size(rng);
      o.phys_bounds.min[1] = yloc + yloc_offs;
      o.phys_bounds.max[1] = o.phys_bounds.min[1] + rand_size(rng);
      o.vel[0] = rand_vel(rng);
      o.vel[1] = rand_vel(rng);

      root.insert(o);
    }
  }

  // A bunch of possibly fast-moving objects in pretty random locations.
  for (int i = 0; i < 400; ++i) {
    uniform_random rand_loc(-1000000,1000000);
    uniform_random rand_vel(-10000,10000);
    int64_t xloc = rand_loc(rng);
    int64_t yloc = rand_loc(rng);

    moving_object<2> o;
    o.id = next_id++;
    o.phys_bounds.min[0] = xloc;
    o.phys_bounds.max[0] = o.phys_bounds.min[0] + rand_size(rng);
    o.phys_bounds.min[1] = yloc;
    o.phys_bounds.max[1] = o.phys_bounds.min[1] + rand_size(rng);
    o.vel[0] = rand_vel(rng);
    o.vel[1] = rand_vel(rng);

    root.insert(o);
  }
}

void do_3d_test_scenario(tree_node<3>& root) {
  object_id next_id = 0;
  root.children.clear();
  root.stuff_here.clear();
  for (int dim = 0; dim < 6; ++dim) {
    root.bounds.min[dim] = 0;
    root.bounds.max[dim] = 1;
  }
  large_fast_noncrypto_rng rng(0);
  uniform_random rand_size(1000,3000);
  
  // A bunch of flat, maybe angled, clumps near the 'ground', all objects going pretty slow.
  for (int i = 0; i < 100; ++i) {
    uniform_random rand_loc(-1000000,1000000);
    uniform_random rand_zloc(-10000,10000);
    uniform_random rand_slope_times_100(-100,100);
    uniform_random rand_square_root_num_objects_times_100(40,300);
    int64_t xloc = rand_loc(rng);
    int64_t yloc = rand_loc(rng);
    int64_t zloc = rand_zloc(rng);
    int64_t xslope_times_100 = rand_slope_times_100(rng);
    int64_t yslope_times_100 = rand_slope_times_100(rng);
    int64_t square_root_num_objects_times_100 = rand_square_root_num_objects_times_100(rng);
    for (int j = 0; j < (square_root_num_objects_times_100 * square_root_num_objects_times_100) / 100; ++j) {
      uniform_random rand_obj_loc(-square_root_num_objects_times_100 * 100, square_root_num_objects_times_100 * 100);
      uniform_random rand_vel(-100,100);

      int64_t xloc_offs = rand_obj_loc(rng);
      int64_t yloc_offs = rand_obj_loc(rng);
      moving_object<3> o;
      o.id = next_id++;
      o.phys_bounds.min[0] = xloc + xloc_offs;
      o.phys_bounds.max[0] = o.phys_bounds.min[0] + rand_size(rng);
      o.phys_bounds.min[1] = yloc + yloc_offs;
      o.phys_bounds.max[1] = o.phys_bounds.min[1] + rand_size(rng);
      o.phys_bounds.min[2] = zloc + (xloc_offs * xslope_times_100 / 100) + (yloc_offs * yslope_times_100 / 100);
      o.phys_bounds.max[2] = o.phys_bounds.min[2] + rand_size(rng);
      o.vel[0] = rand_vel(rng);
      o.vel[1] = rand_vel(rng);
      o.vel[2] = rand_vel(rng);
      
      root.insert(o);
    }
  }

  // A bunch of possibly fast-moving objects in pretty random locations.
  for (int i = 0; i < 400; ++i) {
    uniform_random rand_loc(-1000000,1000000);
    uniform_random rand_zloc(-30000,500000);
    uniform_random rand_vel(-10000,10000);
    int64_t xloc = rand_loc(rng);
    int64_t yloc = rand_loc(rng);
    int64_t zloc = rand_zloc(rng);

    moving_object<3> o;
    o.id = next_id++;
    o.phys_bounds.min[0] = xloc;
    o.phys_bounds.max[0] = o.phys_bounds.min[0] + rand_size(rng);
    o.phys_bounds.min[1] = yloc;
    o.phys_bounds.max[1] = o.phys_bounds.min[1] + rand_size(rng);
    o.phys_bounds.min[2] = zloc;
    o.phys_bounds.max[2] = o.phys_bounds.min[2] + rand_size(rng);
    o.vel[0] = rand_vel(rng);
    o.vel[1] = rand_vel(rng);
    o.vel[2] = rand_vel(rng);
    
    root.insert(o);
  }
}


static void mainLoop (std::string /*scenario*/)
{
  SDL_Event event;
  int done = 0;
  int p_mode = 0;
large_fast_noncrypto_rng rng(time(NULL));

  uniform_random rand_vel(-100,100);
  uniform_random rand_min(-100,70);
  uniform_random rand_size(10,30);
  
  
//  vector3<polygon_int_type> velocity(1,2,3);
int frame = 0;

  tree_node<2> root;
  for (int dim = 0; dim < 4; ++dim) {
    root.bounds.min[dim] = 0;
    root.bounds.max[dim] = 1;
  }
  
  //bool draw_endp = false;
  //bool draw_coll_stuff = false;
  //bool draw_normals = false;
  //bool draw_poly = true;
  //bool use_foo1 = false;
  int insert_objects = 0;
  object_id next_id = 1;
  
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
          if(event.key.keysym.sym == SDLK_z) insert_objects ++;
          if(event.key.keysym.sym == SDLK_x) insert_objects += 50;
          if(event.key.keysym.sym == SDLK_c) insert_objects += 2500;
          if(event.key.keysym.sym == SDLK_a) do_2d_test_scenario(root);
          //if(event.key.keysym.sym == SDLK_z) draw_poly = !draw_poly;
          //if(event.key.keysym.sym == SDLK_x) draw_normals = !draw_normals;
          //if(event.key.keysym.sym == SDLK_c) draw_endp = !draw_endp;
          //if(event.key.keysym.sym == SDLK_v) draw_coll_stuff = !draw_coll_stuff;
          //if(event.key.keysym.sym == SDLK_b) use_foo1 = !use_foo1;
          //if(event.key.keysym.sym == SDLK_q) ++velocity[X];
          //if(event.key.keysym.sym == SDLK_a) --velocity[X];
          //if(event.key.keysym.sym == SDLK_w) ++velocity[Y];
          //if(event.key.keysym.sym == SDLK_s) --velocity[Y];
          //if(event.key.keysym.sym == SDLK_e) ++velocity[Z];
          //if(event.key.keysym.sym == SDLK_d) --velocity[Z];
          //if(event.key.keysym.sym == SDLK_r) obstacle.translate(vector3<polygon_int_type>(1,0,0));
          //if(event.key.keysym.sym == SDLK_f) obstacle.translate(vector3<polygon_int_type>(-1,0,0));
          //if(event.key.keysym.sym == SDLK_t) obstacle.translate(vector3<polygon_int_type>(0,1,0));
          //if(event.key.keysym.sym == SDLK_g) obstacle.translate(vector3<polygon_int_type>(0,-1,0));
          //if(event.key.keysym.sym == SDLK_y) obstacle.translate(vector3<polygon_int_type>(0,0,1));
          //if(event.key.keysym.sym == SDLK_h) obstacle.translate(vector3<polygon_int_type>(0,0,-1));
          //if(event.key.keysym.sym == SDLK_r) ++view_dist;
          //if(event.key.keysym.sym == SDLK_f) --view_dist;
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
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    gluPerspective(80, 1, 100000, 10000000);
    gluLookAt(0+2500000*std::cos(double(frame) / 200),0+2500000*std::sin(double(frame) / 200),1500000,0,0,0,0,0,1);

    draw_node(root);
    
    int before_GL = SDL_GetTicks();
    
    glFinish();	
    SDL_GL_SwapBuffers();
   
    int before_processing = SDL_GetTicks();
    
    //doing stuff code here
    for ( ; insert_objects > 0; --insert_objects) {
      moving_object<2> o;
      o.id = next_id++;
      for (int i = 0; i < 2; ++i) {
        o.phys_bounds.min[i] = rand_min(rng);
        o.phys_bounds.max[i] = o.phys_bounds.min[i] + rand_size(rng);
        o.vel[i] = rand_vel(rng);
      }
      root.insert(o);
      print_nodecount(root);
    }
	++frame;
    
    
    int after = SDL_GetTicks();
    //std::cerr << (after - before_processing) << ", " << (before_GL - before_drawing) << ", " << (before_processing - before_GL) << "\n";

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
	gluPerspective(90, 1, 10, 1000);
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