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


#ifndef LASERCAKE_POLYGON_COLLISION_DETECTION_HPP__
#define LASERCAKE_POLYGON_COLLISION_DETECTION_HPP__

#include <cassert>
#include <vector>
#include <array>

#include "utils.hpp"

// 64 bits though these ints are, you can't really do collision detection with them except for things
//  with a max distance of about 14 bits between any of the parts of them.

struct polygon_collision_info_cache {
  polygon_collision_info_cache():is_valid(false){}
  bool is_valid;
  vector3<int64_t> translation_amount;
  int64_t denom;
  int64_t a_times_denom;
  int64_t b_times_denom;
  int amount_twisted;
  std::vector<vector3<int64_t>> adjusted_vertices;
};

struct bounding_box {
  bounding_box():is_anywhere(false){}
  bounding_box(vector3<int64_t> loc):is_anywhere(true),min(loc),max(loc){}
  bounding_box(vector3<int64_t> min, vector3<int64_t> max):is_anywhere(true),min(min),max(max){}
  
  bool is_anywhere;
  vector3<int64_t> min, max;
  
  bool contains(vector3<int64_t> const& v)const;
  bool overlaps(bounding_box const& o)const;
  void combine_with(bounding_box const& o);
};

struct line_segment {
  line_segment(std::array<vector3<int64_t>, 2> ends):ends(ends){}
  line_segment(vector3<int64_t> end1, vector3<int64_t> end2):ends({{end1, end2}}){}
  std::array<vector3<int64_t>, 2> ends;
  
  void translate(vector3<int64_t> t);
  bounding_box bounds()const;
};

struct convex_polygon {
  // The structure simply trusts that you will provide a convex, coplanar sequence of points. Failure to do so will result in undefined behavior.
  void setup_cache_if_needed()const;
  convex_polygon(std::vector<vector3<int64_t>> const& vertices):vertices(vertices){ assert(vertices.size() >= 3); }
  polygon_collision_info_cache const& get_cache()const { return cache; }
  std::vector<vector3<int64_t>> const& get_vertices()const { return vertices; }
  void translate(vector3<int64_t> t);
  bounding_box bounds()const;
private:
  std::vector<vector3<int64_t>> vertices;
  mutable polygon_collision_info_cache cache;
};

class shape {
  shape(                                       ): bounds_cache_is_valid(false)                {}
  shape(               line_segment const& init): bounds_cache_is_valid(false)                { segments.push_back(init); }
  shape(             convex_polygon const& init): bounds_cache_is_valid(false)                { polygons.push_back(init); }
  shape(std::vector<convex_polygon> const& init): bounds_cache_is_valid(false), polygons(init){}
  shape(               bounding_box const& init);
  
  shape(shape const& o):segments(o.segments),polygons(o.polygons),bounds_cache(o.bounds_cache),bounds_cache_is_valid(o.bounds_cache_is_valid) {}
  
  void translate(vector3<int64_t> t);
  
  bool intersects(shape const& other)const;
  bounding_box bounds()const;
  
  std::vector<  line_segment> const& get_segments()const { return segments; }
  std::vector<convex_polygon> const& get_polygons()const { return polygons; }
private:
  std::vector<  line_segment> segments;
  std::vector<convex_polygon> polygons;
  
  mutable bounding_box bounds_cache;
  mutable bool bounds_cache_is_valid;
};

/*bool intersects(line_segment l, convex_polygon const& p);
bool intersects(convex_polygon const& p1, convex_polygon const& p2);
bool intersects(line_segment const& l, std::vector<convex_polygon> const& ps);
bool intersects(std::vector<convex_polygon> const& ps1, std::vector<convex_polygon> const& ps2);*/

#endif
