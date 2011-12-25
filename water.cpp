
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <map>
#include <set>
#include <array>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <boost/utility.hpp>
#include <iostream>
#include <inttypes.h>

using std::map;
using std::unordered_map;
using std::unordered_set;
using std::pair;
using std::make_pair;
using std::set;
using std::vector;
using std::array;

class bounds_checked_int {
public:
	bounds_checked_int():value(0){}
	bounds_checked_int(int value):value(value){}
	bounds_checked_int &operator=(int other) { value = other; return *this; }
	bounds_checked_int operator+(int other)const {
		assert((int64_t)value + (int64_t)other < (1LL << 31));
		assert((int64_t)value + (int64_t)other > -(1LL << 31));
		return bounds_checked_int(value + other);
	}
	void operator+=(int other) {
		assert((int64_t)value + (int64_t)other < (1LL << 31));
		assert((int64_t)value + (int64_t)other > -(1LL << 31));
		value += other;
	}
	bounds_checked_int& operator++() {
		*this += 1; return *this;
	}
	bounds_checked_int operator++(int) {
		bounds_checked_int result(value);
		*this += 1;
		return result;
	}
	bounds_checked_int& operator--() {
		*this -= 1; return *this;
	}
	bounds_checked_int operator--(int) {
		bounds_checked_int result(value);
		*this -= 1;
		return result;
	}
	bounds_checked_int operator-(int other)const {
		assert((int64_t)value - (int64_t)other < (1LL << 31));
		assert((int64_t)value - (int64_t)other > -(1LL << 31));
		return bounds_checked_int(value - other);
	}
	void operator-=(int other) {
		assert((int64_t)value - (int64_t)other < (1LL << 31));
		assert((int64_t)value - (int64_t)other > -(1LL << 31));
		value -= other;
	}
	bounds_checked_int operator*(int other)const {
		assert((int64_t)value * (int64_t)other < (1LL << 31));
		assert((int64_t)value * (int64_t)other > -(1LL << 31));
		return bounds_checked_int(value * other);
	}
	void operator*=(int other) {
		assert((int64_t)value * (int64_t)other < (1LL << 31));
		assert((int64_t)value * (int64_t)other > -(1LL << 31));
		value *= other;
	}
	bounds_checked_int operator/(int other)const {
		return bounds_checked_int(value / other);
	}
	void operator/=(int other) {
		value /= other;
	}
	operator int()const{ return value; }
private:
	int value;
};

//typedef bounds_checked_int scalar_type;
//typedef int64_t scalar_type;
typedef int scalar_type;

int divide_rounding_towards_zero(scalar_type dividend, scalar_type divisor)
{
	assert(divisor != 0);
	int abs_result = std::abs(dividend) / std::abs(divisor);
	if ((dividend > 0 && divisor > 0) || (dividend < 0 && divisor < 0)) return abs_result;
	else return -abs_result;
}

int sign(int number)
{
	if (number > 0) return 1;
	if (number < 0) return -1;
	return 0;
}


class vector3 {
public:
	scalar_type x, y, z;
	vector3():x(0),y(0),z(0){}
	vector3(int x, int y, int z):x(x),y(y),z(z){}
	vector3 operator+(vector3 const& other)const {
		return vector3(x + other.x, y + other.y, z + other.z);
	}
	void operator+=(vector3 const& other) {
		x += other.x; y += other.y; z += other.z;
	}
	vector3 operator-(vector3 const& other)const {
		return vector3(x - other.x, y - other.y, z - other.z);
	}
	void operator-=(vector3 const& other) {
		x -= other.x; y -= other.y; z -= other.z;
	}
	vector3 operator*(int other)const {
		return vector3(x * other, y * other, z * other);
	}
	void operator*=(int other) {
		x *= other; y *= other; z *= other;
	}
	vector3 operator/(int other)const {
		return vector3(divide_rounding_towards_zero(x, other), divide_rounding_towards_zero(y, other), divide_rounding_towards_zero(z, other));
	}
	void operator/=(int other) {
		x = divide_rounding_towards_zero(x, other); y = divide_rounding_towards_zero(y, other); z = divide_rounding_towards_zero(z, other);
	}
	vector3 operator-()const { // unary minus
		return vector3(-x, -y, -z);
	}
	bool operator==(vector3 const& other)const {return x == other.x && y == other.y && z == other.z; }
	bool operator!=(vector3 const& other)const {return x != other.x || y != other.y || z != other.z; }
	scalar_type dot(vector3 const& other)const {
		return x * other.x + y * other.y + z * other.z;
	}
	scalar_type magnitude()const { return std::sqrt((int)dot(*this));} // TODO why is it converted to an int and how can I fix that
	bool magnitude_is_less_than(scalar_type amount)const { return dot(*this) < amount * amount; }
	bool magnitude_is_greater_than(scalar_type amount)const { return dot(*this) > amount * amount; }
	bool operator<(vector3 const& other)const { return (x < other.x) || ((x == other.x) && ((y < other.y) || ((y == other.y) && (z < other.z)))); }
};

typedef vector3 one_tile_direction_vector;
typedef vector3 location;

vector3 project_onto_cardinal_direction(vector3 src, one_tile_direction_vector dir) {
  return vector3(src.x * std::abs(dir.x), src.y * std::abs(dir.y), src.z * std::abs(dir.z));
}

namespace std {
  template<> struct hash<location> {
    inline size_t operator()(location const& loc) const {
      size_t seed = 0;
      boost::hash_combine(seed, loc.x);
      boost::hash_combine(seed, loc.y);
      boost::hash_combine(seed, loc.z);
      return seed;
    }
  };
}

#define MAX_X 100
#define MAX_Y 100
#define MAX_Z 100

namespace hacky_vector_indexing_internals {
  // the order of this must be in sync with the order of cardinal_direction_vectors
  int cardinal_direction_vector_to_index(one_tile_direction_vector v) {
         if (v.x == -1) return 0;
    else if (v.y == -1) return 1;
    else if (v.z == -1) return 2;
    else if (v.x ==  1) return 3;
    else if (v.y ==  1) return 4;
    else if (v.z ==  1) return 5;
    else                  assert(false);
  }
}

const one_tile_direction_vector xunitv = one_tile_direction_vector(1, 0, 0);
const one_tile_direction_vector yunitv = one_tile_direction_vector(0, 1, 0);
const one_tile_direction_vector zunitv = one_tile_direction_vector(0, 0, 1);
// the order of this must be in sync with the order of hacky_vector_indexing_internals::cardinal_direction_vector_to_index
const int NUM_CARDINAL_DIRECTIONS = 6;
const one_tile_direction_vector cardinal_direction_vectors[NUM_CARDINAL_DIRECTIONS] = { -xunitv, -yunitv, -zunitv, xunitv, yunitv, zunitv };

void incr_location(location &foo){
	++foo.x;
	if (foo.x >= MAX_X) { foo.x = 0; ++foo.y; }
	if (foo.y >= MAX_Y) { foo.y = 0; ++foo.z; }
}
void decr_location(location &foo){
	--foo.x;
	if (foo.x < 0) { foo.x = MAX_X - 1; --foo.y; }
	if (foo.y < 0) { foo.y = MAX_Y - 1; --foo.z; }
}
void incr_direction(one_tile_direction_vector &foo){
	++foo.x;
	if (foo.x == 0 && foo.y == 0 && foo.z == 0) ++foo.x;
	if (foo.x >= 2) { foo.x = -1; ++foo.y; }
	if (foo.y >= 2) { foo.y = -1; ++foo.z; }
}

#define EACH_LOCATION(varname) location varname(0,0,0); varname.z < MAX_Z; incr_location(varname)
#define EACH_LOCATION_REVERSE(varname) location varname(MAX_X - 1,MAX_Y - 1,MAX_Z - 1); varname.z >= 0; decr_location(varname)
#define EACH_DIRECTION(varname) one_tile_direction_vector varname(-1,-1,-1); varname.z < 2; incr_direction(varname)
#define EACH_CARDINAL_DIRECTION(varname) const one_tile_direction_vector varname : cardinal_direction_vectors


template<typename value_type> class value_for_each_cardinal_direction {
public:
  value_for_each_cardinal_direction& operator=(value_for_each_cardinal_direction const& other){
    for(int dir_idx=0; dir_idx < NUM_CARDINAL_DIRECTIONS; ++dir_idx) {
      data[dir_idx] = other.data[dir_idx];
    }
    return *this;
  }
  value_for_each_cardinal_direction(value_type initial_value) {
    for(int dir_idx=0; dir_idx < NUM_CARDINAL_DIRECTIONS; ++dir_idx) {
      data[dir_idx] = initial_value;
    }
  }
  value_type      & operator[](one_tile_direction_vector dir)      { return data[hacky_vector_indexing_internals::cardinal_direction_vector_to_index(dir)]; }
  value_type const& operator[](one_tile_direction_vector dir)const { return data[hacky_vector_indexing_internals::cardinal_direction_vector_to_index(dir)]; }
  value_type      & operator[](int dir_idx)      { return data[dir_idx]; }
  value_type const& operator[](int dir_idx)const { return data[dir_idx]; }
private:
  typedef array<value_type, NUM_CARDINAL_DIRECTIONS> internal_array;
public:
  typename internal_array::iterator begin() { return data.begin(); }
  typename internal_array::iterator end() { return data.end(); }
  typename internal_array::const_iterator cbegin()const { return data.cbegin(); }
  typename internal_array::const_iterator cend()const { return data.cend(); }
private:
  internal_array data;
};



/*

Whether tiles are water or not
 -- dictates, at one tile distance --
Whether water tiles are "sticky water" ("fewer than two adjacent tiles are air") or "free water" (at least two adjacent air tiles)
 -- dictates, at one tile distance --
Whether sticky water tiles are "interior water" ("every adjacent tile is sticky water") or "membrane water" ("at least one tile is not a sticky water tile")

This is a one-way dictation - the latter things don't affect the former things at all (until they cause water to actually move, anyway). It's not an exact dictation 

*/


enum tile_contents {
  ROCK,
  WATER,
  AIR
};

const scalar_type precision_factor = 100;
const scalar_type progress_necessary = 5000 * precision_factor; // loosely speaking, the conversion factor between mini-units and entire tiles
const scalar_type min_convincing_speed = 100 * precision_factor;
const vector3 gravity_acceleration(0, 0, -5*precision_factor); // in mini-units per frame squared
const scalar_type friction_amount = 3 * precision_factor;

// as in 1 + d2 (except with the random based at zero, but who cares)
const scalar_type pressure_motion_factor = 80 * precision_factor;
const scalar_type pressure_motion_factor_random = 40 * precision_factor;
const scalar_type extra_downward_speed_for_sticky_water = 100 * precision_factor;

const scalar_type air_resistance_constant = (200000 * precision_factor * precision_factor);
const scalar_type idle_progress_reduction_rate = 100 * precision_factor;
const scalar_type sticky_water_velocity_reduction_rate = 5*precision_factor;

const vector3 idle_water_velocity(0, 0, -min_convincing_speed);

struct water_movement_info {
  vector3 velocity;
  value_for_each_cardinal_direction<scalar_type> progress;
  value_for_each_cardinal_direction<scalar_type> new_progress;
  value_for_each_cardinal_direction<scalar_type> blockage_amount_this_frame;
  
  // Constructing one of these in the default way yields the natural idle state:
  // however we should NOT use std::unordered_map's operator[] to activate it; it should only be created through tiles' activate_water and insert_water.
  water_movement_info():velocity(idle_water_velocity),progress(0),new_progress(0),blockage_amount_this_frame(0){ progress[-zunitv] = progress_necessary; }
  
  // This is not a general-purpose function. Only use it during the move-processing part of update_water.
  void get_completely_blocked(one_tile_direction_vector dir) {
    const scalar_type dp = velocity.dot(dir);
    const scalar_type blocked_velocity = dp - min_convincing_speed;
    if (blocked_velocity > 0) {
      velocity -= dir * blocked_velocity;
    }
  }
  
  bool can_deactivate()const {
    // TODO: does it make sense that we're ignoring the 1-frame-duration variable "blockage_amount_this_frame"?
    for (int dir_idx = 0; dir_idx < NUM_CARDINAL_DIRECTIONS; ++dir_idx) {
      one_tile_direction_vector const& dir = cardinal_direction_vectors[dir_idx];
      if (dir.z < 0) {
        if (progress[dir_idx] != progress_necessary) return false;
      }
      else {
        if (progress[dir_idx] != 0) return false;
      }
    }
    return velocity == idle_water_velocity;
  }
};

typedef int group_number_t;
const group_number_t NO_GROUP = -1;
const group_number_t FIRST_GROUP = 0;

struct tile
{
  tile_contents contents;
  bool is_sticky;
  bool is_interior;
  bool is_sticky_water  ()const{ return contents == WATER &&  is_sticky                ; }
  bool is_free_water    ()const{ return contents == WATER && !is_sticky                ; }
  bool is_interior_water()const{ return contents == WATER &&  is_sticky &&  is_interior; }
  bool is_membrane_water()const{ return contents == WATER &&  is_sticky && !is_interior; }
};

bool out_of_bounds(location loc){return loc.x < 0 || loc.y < 0 || loc.z < 0 || loc.x >= MAX_X || loc.y >= MAX_Y || loc.z >= MAX_Z; }

class tiles_type {
private:
  tile tiles[MAX_X][MAX_Y][MAX_Z];
  void check_stickyness(location loc) {
    if (out_of_bounds(loc)) return;
    tile &t = (*this)[loc];
    if (t.contents == WATER) { // we don't care whether it's marked sticky if it's not water
      int airs = 0;
      for (EACH_CARDINAL_DIRECTION(dir)) {
        if (!out_of_bounds(loc + dir) && (*this)[loc + dir].contents == AIR) ++airs;
      }
      bool should_be_sticky = (airs <= 1);
      if (t.is_sticky != should_be_sticky) {
        t.is_sticky = should_be_sticky;
        if (t.is_sticky) check_interiorness(loc);
        for (EACH_CARDINAL_DIRECTION(dir)) check_interiorness(loc + dir);
      }
    }
  }
  void check_interiorness(location loc) {
    if (out_of_bounds(loc)) return;
    tile &t = (*this)[loc];
    if (t.contents == WATER && t.is_sticky) { // we don't care whether it's marked interior if it's not sticky water
      bool should_be_interior = true;
      for (EACH_CARDINAL_DIRECTION(dir)) {
        if (out_of_bounds(loc + dir) || !(*this)[loc + dir].is_sticky_water()) should_be_interior = false;
      }
      if (t.is_interior != should_be_interior) {
        t.is_interior = should_be_interior;
      }
    }
  }
public:
  unordered_map<location, water_movement_info> active_tiles;
  unordered_set<location> idle_water; // this is currently just a cache for the display...
  
  tile& operator[](location loc){ assert(!out_of_bounds(loc)); return tiles[loc.x][loc.y][loc.z]; }
  tile const& operator[](location loc)const { assert(!out_of_bounds(loc)); return tiles[loc.x][loc.y][loc.z]; }
  
  void delete_water(location loc) {
    assert((*this)[loc].contents == WATER);
    (*this)[loc].contents = AIR;
    active_tiles.erase(loc);
    idle_water.erase(loc);
    
    for (EACH_CARDINAL_DIRECTION(dir)) check_stickyness(loc + dir);
  }
  water_movement_info& insert_water(location loc) {
    assert((*this)[loc].contents == AIR);
    (*this)[loc].contents = WATER;
    
    check_stickyness(loc);
    for (EACH_CARDINAL_DIRECTION(dir)) check_stickyness(loc + dir);
    
    // created water always starts out active.
    return active_tiles[loc]; // inserts it, default-constructed
  }
  void deactivate_water(location loc) {
    auto i = active_tiles.find(loc);
    if (i != active_tiles.end()) {
      active_tiles.erase(i);
      idle_water.insert(loc);
    }
  }
  water_movement_info& activate_water(location loc) {
    assert((*this)[loc].contents == WATER);
    auto i = active_tiles.find(loc);
    if (i == active_tiles.end()) {
      idle_water.erase(loc);
      return active_tiles[loc]; // inserts it, default-constructed
    }
    return i->second;
  }
};

tiles_type tiles;

template<typename Map>
typename Map::mapped_type* find_as_pointer(Map& m, typename Map::key_type const& k) {
  auto i = m.find(k);
  if(i == m.end()) return nullptr;
  else return &(i->second);
}



/*bool is_sticky_water(location loc) {
  if (tiles[loc].contents != WATER) return false;
  int airs = 0;
  for (EACH_CARDINAL_DIRECTION(dir)) {
    if (!out_of_bounds(loc + dir) && tiles[loc + dir].contents == AIR) ++airs;
  }
  if (airs > 1) return false;
  return true;
}

bool is_free_water(location loc) {
  return tiles[loc].contents == WATER && !is_sticky_water(loc);
}

bool is_interior_water(location loc) {
  for (EACH_CARDINAL_DIRECTION(dir)) {
    if (out_of_bounds(loc + dir) || !is_sticky_water(loc + dir)) return false;
  }
  return is_sticky_water(loc);
}

bool is_membrane_water(location loc) {
  return is_sticky_water(loc) && !is_interior_water(loc);
}*/

/*struct water_group {
  int max_tile_z;
  std::set<location> tiles;
  set<pair<location, one_tile_direction_vector> > exit_surfaces; // (sticky water, direction towards something that's either air or free water)
  water_group():max_tile_z(0){}
};*/

struct water_groups_structure {
  unordered_map<location, group_number_t> group_numbers_by_location;
  vector<unordered_set<location> > locations_by_group_number;
  vector<scalar_type> max_z_by_group_number;
};

void collect_membrane(location const& loc, group_number_t group_number, water_groups_structure &result) {
  vector<location> frontier;
  frontier.push_back(loc);
  while(!frontier.empty())
  {
    const location next_loc = frontier.back();
    frontier.pop_back();
    if (result.group_numbers_by_location.find(next_loc) == result.group_numbers_by_location.end()) {
      result.group_numbers_by_location.insert(make_pair(next_loc, group_number));
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const location adj_loc = next_loc + dir;
        if (!out_of_bounds(adj_loc) && tiles[adj_loc].is_membrane_water()) {
          frontier.push_back(adj_loc);
        }
      }
    }
  }
}

// the argument must start out default-initialized
void compute_groups_that_need_to_be_considered(water_groups_structure &result) {
  group_number_t next_membrane_number = FIRST_GROUP;
  for (auto const& p : tiles.active_tiles) {
    location const& loc = p.first;
    if (tiles[loc].is_membrane_water() && result.group_numbers_by_location.find(loc) == result.group_numbers_by_location.end()) {
      collect_membrane(loc, next_membrane_number, result);
      ++next_membrane_number;
    }
  }
  
  // Make a list of locations, sorted by X then Y then Z
  vector<pair<location, group_number_t> > columns; columns.reserve(result.group_numbers_by_location.size());
  columns.insert(columns.end(), result.group_numbers_by_location.begin(), result.group_numbers_by_location.end());
  std::sort(columns.begin(), columns.end()); // TODO: don't rely on location's < to be x-then-y-then-z
  
  group_number_t containing_group;
  for (pair<location, group_number_t> const& p : columns) {
    if (!out_of_bounds(p.first - zunitv) && tiles[p.first - zunitv].is_sticky_water()) {
      // The tile we've just hit is the boundary of a bubble in the larger group (or it's extra bits of the outer shell, in which case the assignment does nothing).
      result.group_numbers_by_location[p.first] = containing_group;
    }
    else {
      // We're entering a group from the outside, which means it functions as a containing group.
      containing_group = p.second;
    }
  }
  
  // Now we might have some sort of group numbers like "1, 2, 7" because they've consolidated in essentially a random order. Convert them to a nice set of indices like 0, 1, 2.
  map<group_number_t, group_number_t> key_compacting_map;
  group_number_t next_group_number = FIRST_GROUP;
  for (pair<const location, group_number_t> &p : result.group_numbers_by_location) {
    if (group_number_t *new_number = find_as_pointer(key_compacting_map, p.second)) {
      p.second = *new_number;
    }
    else {
      p.second = key_compacting_map[p.second] = next_group_number++; // Bwa ha ha ha ha.
      result.locations_by_group_number.push_back(unordered_set<location>());
      result.max_z_by_group_number.push_back(p.first.z);
    }
    result.locations_by_group_number[p.second].insert(p.first);
    result.max_z_by_group_number[p.second] = std::max(result.max_z_by_group_number[p.second], p.first.z);
  }
}


/*void mark_water_group_that_includes(location loc, int group_number, map<int, water_group>& groups) {
  set<location> frontier;
  frontier.insert(loc);
  groups.insert(make_pair(group_number, water_group()));
  while(!frontier.empty())
  {
    const location next_loc = *(frontier.begin());
    frontier.erase(frontier.begin());
    
    tiles[next_loc].water_group_number = group_number;
    groups[group_number].tiles.insert(next_loc);
    if (next_loc.z > groups[group_number].max_tile_z) groups[group_number].max_tile_z = next_loc.z;
    
    if (is_sticky_water(next_loc)) {
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const location adj_loc = next_loc + dir;
        if (out_of_bounds(adj_loc)) continue;
        tile &adj_tile = tiles[adj_loc];
        if (adj_tile.contents == WATER && adj_tile.water_group_number != group_number) {
          frontier.insert(adj_loc);
        }
        // Hack? Include tiles connected diagonally, if there's air in between (this makes sure that water using the 'fall off pillars' rule to go into a lake is grouped with the lake)
        if (adj_tile.contents == AIR) {
          for (EACH_CARDINAL_DIRECTION(d2)) {
            if (d2.dot(dir) == 0 && !out_of_bounds(adj_loc + d2) && tiles[adj_loc + d2].contents == WATER && tiles[adj_loc + d2].water_group_number != group_number) {
              frontier.insert(adj_loc + d2);
            }
          }
        }
        if (adj_tile.contents == AIR || (adj_tile.contents == WATER && is_free_water(adj_loc))) {
          groups[group_number].exit_surfaces.insert(std::make_pair(next_loc, dir));
        }
      }
    }
  }
}*/

struct wanted_move {
  location src;
  one_tile_direction_vector dir;
  group_number_t group_number_or_NO_GROUP_for_velocity_movement;
  scalar_type amount_of_the_push_that_sent_us_over_the_threshold;
  scalar_type excess_progress;
  wanted_move(location src,one_tile_direction_vector dir,group_number_t g,scalar_type a,scalar_type e):src(src),dir(dir),group_number_or_NO_GROUP_for_velocity_movement(g),amount_of_the_push_that_sent_us_over_the_threshold(a),excess_progress(e){}
};

  
template<typename Stuff> struct literally_random_access_removable_stuff {
public:
  void add(Stuff stuff) {
    stuffs.push_back(stuff);
    blacklist_list.push_back(false);
  }
  void blacklist(size_t which) {
    ++num_blacklisted;
    blacklist_list[which] = true;
    if (num_blacklisted * 2 > stuffs.size()) {
      purge_blacklisted_stuffs();
    }
  }
  Stuff get_and_blacklist_random() {
    assert(!stuffs.empty());
    size_t idx;
    do { idx = (size_t)(rand()%(stuffs.size())); } while (blacklist_list[idx]);
    Stuff result = stuffs[idx];
    blacklist(idx);
    return result;
  }
  bool empty()const { return stuffs.empty(); }
private:
  vector<Stuff> stuffs;
  vector<bool> blacklist_list;
  size_t num_blacklisted;
  void purge_blacklisted_stuffs() {
    size_t next_insert_idx = 0;
    for (size_t i = 0; i < stuffs.size(); ++i) {
      if (!blacklist_list[i]) {
        stuffs[next_insert_idx] = stuffs[i];
        blacklist_list[next_insert_idx] = false;
        ++next_insert_idx;
      }
    }
    stuffs.erase(stuffs.begin() + next_insert_idx, stuffs.end());
    blacklist_list.erase(blacklist_list.begin() + next_insert_idx, blacklist_list.end());
    num_blacklisted = 0;
  }
};

void update_water() {
  for (auto i = tiles.active_tiles.begin(); i != tiles.active_tiles.end(); ++i) {
    tile const& t = tiles[i->first];
    assert(t.contents == WATER);
    for (scalar_type &np : i->second.new_progress) np = 0;
    
    if (t.is_sticky_water()) {
      vector3 &vel_ref = i->second.velocity;
      const vector3 current_velocity_wrongness = vel_ref - idle_water_velocity;
      if (current_velocity_wrongness.magnitude_is_less_than(sticky_water_velocity_reduction_rate)) {
        vel_ref = idle_water_velocity;
      }
      else {
        vel_ref -= current_velocity_wrongness * sticky_water_velocity_reduction_rate / current_velocity_wrongness.magnitude();
      }
    }
  }
  
  water_groups_structure groups;
  compute_groups_that_need_to_be_considered(groups);

  for (group_number_t group_number = FIRST_GROUP; group_number < (group_number_t)groups.locations_by_group_number.size(); ++group_number) {
    for (location const& loc : groups.locations_by_group_number[group_number]) {
      for (int dir_idx = 0; dir_idx < NUM_CARDINAL_DIRECTIONS; ++dir_idx) {
        one_tile_direction_vector const& dir = cardinal_direction_vectors[dir_idx];
        const location dst_loc = loc + dir;
        if (!out_of_bounds(dst_loc)) {
          tile const& dst_tile = tiles[dst_loc];
          if (!dst_tile.is_sticky_water() && dst_tile.contents != ROCK) { // i.e. is air or free water. Those exclusions aren't terribly important, but it'd be slightly silly to remove either of them (and we currently rely on both exclusions to make the idle state what it is.)
            const double pressure = ((double)groups.max_z_by_group_number[group_number] - 0.5) - ((double)loc.z + 0.5*dir.z); // proportional to depth, assuming side surfaces are at the middle of the side. This is less by 1.0 than it naturally should be, to prevent water that should be stable (if unavoidably uneven by 1 tile or less) from fluctuating.
            if (pressure > 0) {
              tiles.activate_water(loc).new_progress[dir_idx] += (scalar_type)((pressure_motion_factor + (rand()%pressure_motion_factor_random)) * std::sqrt(pressure));
            }
          }
        }
      }
    }
  }
  
  for (auto i = tiles.active_tiles.begin(); i != tiles.active_tiles.end(); ++i) {
    location const& loc = i->first;
    tile const& t = tiles[i->first];
    assert(t.contents == WATER);
    if (t.is_sticky_water()) {
      i->second.new_progress[-zunitv] += extra_downward_speed_for_sticky_water;
    }
    else {
      vector3 &vel_ref = i->second.velocity;
      vel_ref += gravity_acceleration;
      
      // Slight air resistance proportional to the square of the velocity (has very little effect at our current 20x20x20 scale; mostly there to make a natural cap velocity for falling water)
      vel_ref -= (vel_ref * vel_ref.magnitude()) / air_resistance_constant;
      
      // Relatively large friction against the ground
      for (int dir_idx = 0; dir_idx < NUM_CARDINAL_DIRECTIONS; ++dir_idx) {
        one_tile_direction_vector const& dir = cardinal_direction_vectors[dir_idx];
        if (i->second.blockage_amount_this_frame[dir_idx] > 0) {
          vector3 copy_stationary_in_blocked_direction = vel_ref; copy_stationary_in_blocked_direction -= project_onto_cardinal_direction(copy_stationary_in_blocked_direction, dir);
          if (copy_stationary_in_blocked_direction.magnitude_is_less_than(friction_amount)) {
            vel_ref -= copy_stationary_in_blocked_direction;
          }
          else {
            vel_ref -= copy_stationary_in_blocked_direction * friction_amount / copy_stationary_in_blocked_direction.magnitude();
          }
        }
      }
      
      for (int dir_idx = 0; dir_idx < NUM_CARDINAL_DIRECTIONS; ++dir_idx) {
        one_tile_direction_vector const& dir = cardinal_direction_vectors[dir_idx];
        
        const scalar_type dp = vel_ref.dot(dir);
        if (dp > 0) i->second.new_progress[dir_idx] += dp;

        // Water that's blocked, but can go around in a diagonal direction, makes "progress" towards all those possible directions (so it'll go in a random direction if it could go around in several different diagonals, without having to 'choose' one right away and gain velocity only in that direction). The main purpose of this is to make it so that water doesn't stack nicely in pillars, or sit still on steep slopes.
        const location dst_loc = loc + dir;
        if (!out_of_bounds(dst_loc) && tiles[dst_loc].contents == AIR) {
          for (EACH_CARDINAL_DIRECTION(d2)) {
            const location diag_loc = dst_loc + d2;
            if (i->second.blockage_amount_this_frame[d2] > 0 && d2.dot(dir) == 0 &&  !out_of_bounds(diag_loc) && tiles[diag_loc].contents == AIR) {
              i->second.new_progress[dir_idx] += i->second.blockage_amount_this_frame[d2];
            }
          }
        }
      }
    }
      
    // This always happens, even if water becomes sticky right after it gets blocked.
    for (scalar_type &bb : i->second.blockage_amount_this_frame) bb = 0;
  }
  
  vector<wanted_move> wanted_moves;
  for (auto i = tiles.active_tiles.begin(); i != tiles.active_tiles.end(); ++i) {
    tile const& t = tiles[i->first];
    assert(t.contents == WATER);
    for (int dir_idx = 0; dir_idx < NUM_CARDINAL_DIRECTIONS; ++dir_idx) {
      one_tile_direction_vector const& dir = cardinal_direction_vectors[dir_idx];
      scalar_type &progress_ref = i->second.progress[dir_idx];
      const scalar_type new_progress = i->second.new_progress[dir_idx];
      if (i->second.new_progress[dir_idx] == 0) {
        if (progress_ref < idle_progress_reduction_rate) progress_ref = 0;
        else progress_ref -= idle_progress_reduction_rate;
      }
      else {
        assert(new_progress >= 0);
        assert(progress_ref >= 0);
        assert(progress_ref <= progress_necessary);
        progress_ref += new_progress;
        if (progress_ref > progress_necessary) {
          wanted_moves.push_back(wanted_move(i->first, dir, t.is_sticky_water() ? groups.group_numbers_by_location[i->first] : NO_GROUP, new_progress, progress_ref - progress_necessary));
          progress_ref = progress_necessary;
        }
      }
    }
  }
  
  std::random_shuffle(wanted_moves.begin(), wanted_moves.end());
  std::set<location> disturbed_tiles;

  vector<map<scalar_type, literally_random_access_removable_stuff<location> > > tiles_by_z_location_by_group_number(groups.locations_by_group_number.size());
  
  for (group_number_t group_number = FIRST_GROUP; group_number < (group_number_t)groups.locations_by_group_number.size(); ++group_number) {
    unordered_set<location> nearby_free_waters;
    for (location const& loc : groups.locations_by_group_number[group_number]) {
      tiles_by_z_location_by_group_number[group_number][loc.z].add(loc);
      for (EACH_CARDINAL_DIRECTION(dir)) {
        const location dst_loc = loc + dir;
        if (out_of_bounds(dst_loc)) continue;
        if (tiles[dst_loc].is_free_water()) {
          nearby_free_waters.insert(dst_loc);
        }
        // Hack? Include tiles connected diagonally, if there's air in between (this makes sure that water using the 'fall off pillars' rule to go into a lake is grouped with the lake)
        if (tiles[dst_loc].contents == AIR) {
          for (EACH_CARDINAL_DIRECTION(d2)) {
            if (d2.dot(dir) == 0) {
              const location diag_loc = dst_loc + d2;
              if (!out_of_bounds(diag_loc) && tiles[diag_loc].is_free_water()) {
                nearby_free_waters.insert(diag_loc);
              }
            }
          }
        }
      }
    }
    for (location const& loc : nearby_free_waters) {
      tiles_by_z_location_by_group_number[group_number][loc.z].add(loc);
    }
  }
  
  for (const wanted_move move : wanted_moves) {
    const location dst = move.src + move.dir;
    tile &src_tile = tiles[move.src];
    tile src_copy = src_tile; // only for use in gdb
    // in certain situations we shouldn't try to move water more than once
    if (disturbed_tiles.find(move.src) != disturbed_tiles.end()) continue;
    // anything where the water was yanked away should have been marked "disturbed"
    assert(src_tile.contents == WATER);
    
    auto fbdkopsw = tiles.active_tiles.find(move.src);
    assert(fbdkopsw != tiles.active_tiles.end());
    water_movement_info &src_water = fbdkopsw->second;
    scalar_type& progress_ref = src_water.progress[move.dir];
    
    if (!out_of_bounds(dst) && tiles[dst].contents == AIR) {
      progress_ref -= progress_necessary;
      water_movement_info &dst_water = tiles.insert_water(dst);
      
      if (move.group_number_or_NO_GROUP_for_velocity_movement == NO_GROUP) {
        dst_water = src_water;
        tiles.delete_water(move.src);
        disturbed_tiles.insert(move.src);
      }
      else {
        // the teleported tile starts with zero everything, for some reason (TODO: why should it be like this?)
        dst_water.velocity = vector3(0,0,0); dst_water.progress[-zunitv] = 0;
        
        // Pick a random top tile.
        map<scalar_type, literally_random_access_removable_stuff<location> > m = tiles_by_z_location_by_group_number[move.group_number_or_NO_GROUP_for_velocity_movement];
        location chosen_top_tile;
        while (true) {
          assert(!m.empty());
          map<scalar_type, literally_random_access_removable_stuff<location> >::iterator l = boost::prior(m.end());
          if (l->second.empty()) m.erase(l);
          else {
            chosen_top_tile = l->second.get_and_blacklist_random();
            if (disturbed_tiles.find(chosen_top_tile) == disturbed_tiles.end()) break;
          }
        }
        // Since the source tile is still water, there always must be a real tile to choose
        
        tiles.delete_water(chosen_top_tile);
        disturbed_tiles.insert(chosen_top_tile);
      }
      
      // If a tile moves, we're now content to assume that it was moving because it had a realistic velocity in that direction, so we should continue with that assumption.
      const scalar_type amount_of_new_vel_in_movement_dir = dst_water.velocity.dot(move.dir);
      const scalar_type deficiency_of_new_vel_in_movement_dir = move.amount_of_the_push_that_sent_us_over_the_threshold - amount_of_new_vel_in_movement_dir;
      if (deficiency_of_new_vel_in_movement_dir > 0) {
        dst_water.velocity += move.dir * deficiency_of_new_vel_in_movement_dir;
      }
      // Also, don't lose movement to rounding error during progress over multiple tiles:
      dst_water.progress[move.dir] = std::min(move.excess_progress, progress_necessary);
    }
    else {
      // we're blocked
      src_water.blockage_amount_this_frame[move.dir] = move.excess_progress;
      
      bool oob = out_of_bounds(dst);
      tile const* dst_tile_ptr = oob ? nullptr : &tiles[dst];
      
      if (!oob && dst_tile_ptr->contents == WATER) {
        if (move.group_number_or_NO_GROUP_for_velocity_movement == NO_GROUP) {
          if (dst_tile_ptr->is_sticky_water()) {
            //TODO... right now, same as rock. Dunno if it should be different.
            src_water.get_completely_blocked(move.dir);
          }
          else {
            water_movement_info &dst_water = tiles.activate_water(dst);
            
            const vector3 vel_diff = src_water.velocity - dst_water.velocity;
            const vector3 exchanged_velocity = project_onto_cardinal_direction(vel_diff, move.dir) / 2;
            src_water.velocity -= exchanged_velocity;
            dst_water.velocity += exchanged_velocity;
            // Since we *don't* move the 'progress' back then they will keep colliding, which means their velocity keeps getting equalized - that's probably a good thing.
          }
        }
        else {
          // we tried to move due to pressure, but bumped into free water! Turn our movement into velocity, at some conversion factor.
          tiles.activate_water(dst).velocity += (move.dir * move.excess_progress) / 10;
        }
      }
      else if (oob || dst_tile_ptr->contents == ROCK) {
        // TODO figure out what to actually do about the fact that water can become sticky while having lots of progress.
        //assert(move.group_number_or_NO_GROUP_for_velocity_movement == NO_GROUP);
        src_water.get_completely_blocked(move.dir);
      }
      else assert(false);
    }
  }
  
  for (auto i = tiles.active_tiles.begin(); i != tiles.active_tiles.end(); ) {
    if (i->second.can_deactivate()) {
      location const& loc = i->first;
      ++i;
      tiles.deactivate_water(loc);
    }
    else ++i;
  }
}

