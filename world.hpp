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

#ifndef LASERCAKE_WORLD_HPP__
#define LASERCAKE_WORLD_HPP__

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
#include <boost/range/iterator_range.hpp>
#include <iostream>
#include <inttypes.h>
#include <functional>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/variant.hpp>
#include <boost/variant/get.hpp>
#include <memory>
#include <boost/shared_ptr.hpp>

#include "utils.hpp"
#include "polygon_collision_detection.hpp"
#include "bbox_collision_detector.hpp"
#include "tiles.hpp"
#include "input_representation.hpp"
#include "bounds_checked_int.hpp"
#include "world_constants.hpp"

using std::pair;
using std::make_pair;
using std::map;
using std::set;
using std::unordered_map;
using std::unordered_set;
using std::vector;
using std::array;
using boost::shared_ptr;


inline vector3<tile_coordinate> get_containing_tile_coordinates(vector3<fine_scalar> v) {
  return vector3<tile_coordinate>(
    tile_coordinate(v.x / tile_width),
    tile_coordinate(v.y / tile_width),
    tile_coordinate(v.z / tile_height)
  );
}
inline fine_scalar lower_bound_in_fine_units(tile_coordinate c, which_dimension_type which_coordinate) {
  if (which_coordinate == Z) return fine_scalar(c) * tile_height;
  else                       return fine_scalar(c) * tile_width;
}
inline fine_scalar upper_bound_in_fine_units(tile_coordinate c, which_dimension_type which_coordinate) {
  if (which_coordinate == Z) return fine_scalar(c) * tile_height + (tile_height-1);
  else                       return fine_scalar(c) * tile_width + (tile_width-1);
}
inline vector3<fine_scalar> lower_bound_in_fine_units(vector3<tile_coordinate> v) {
  return vector3<fine_scalar>(
    lower_bound_in_fine_units(v[0], 0),
    lower_bound_in_fine_units(v[1], 1),
    lower_bound_in_fine_units(v[2], 2)
  );
}
inline vector3<fine_scalar> upper_bound_in_fine_units(vector3<tile_coordinate> v) {
  return vector3<fine_scalar>(
    upper_bound_in_fine_units(v[0], 0),
    upper_bound_in_fine_units(v[1], 1),
    upper_bound_in_fine_units(v[2], 2)
  );
}

inline bounding_box fine_bounding_box_of_tile(vector3<tile_coordinate> v) {
  return bounding_box(lower_bound_in_fine_units(v), upper_bound_in_fine_units(v));
}

inline bounding_box convert_to_fine_units(tile_bounding_box const& bb) {
  return bounding_box(
    lower_bound_in_fine_units(bb.min),
    lower_bound_in_fine_units(bb.min + bb.size) - vector3<fine_scalar>(1,1,1)
  );
}

inline tile_bounding_box convert_to_smallest_superset_at_tile_resolution(bounding_box const& bb) {
  tile_bounding_box result;
  result.min = get_containing_tile_coordinates(bb.min);
  result.size = get_containing_tile_coordinates(bb.max) + vector3<tile_coordinate>(1,1,1) - result.min;
  return result;
}

inline shape tile_shape(vector3<tile_coordinate> tile) {
  return shape(fine_bounding_box_of_tile(tile));
}


const tile_coordinate world_center_tile_coord = (tile_coordinate(1) << (8*sizeof(tile_coordinate) - 2));
const vector3<tile_coordinate> world_center_tile_coords(world_center_tile_coord, world_center_tile_coord, world_center_tile_coord);
const vector3<fine_scalar> world_center_fine_coords = lower_bound_in_fine_units(world_center_tile_coords);





typedef uint64_t object_identifier;
const object_identifier NO_OBJECT = 0;

struct object_or_tile_identifier {
  object_or_tile_identifier():data_(NO_OBJECT){}
  object_or_tile_identifier(tile_location const& loc):data_(loc){}
  object_or_tile_identifier(object_identifier id):data_(id){}
  tile_location const* get_tile_location()const { return boost::get<tile_location>(&data_); }
  object_identifier const* get_object_identifier()const { return boost::get<object_identifier>(&data_); }
  friend inline size_t hash_value(object_or_tile_identifier const& id) {
    struct hash_visitor : public boost::static_visitor<size_t>
    {
      size_t operator()(tile_location const& i) const {
        return std::hash<tile_location>()(i);
      }
    
      size_t operator()(object_identifier i) const {
        return std::hash<object_identifier>()(i);
      }
    };
    return boost::apply_visitor( hash_visitor(), id.data_ );
  }
  bool operator==(object_or_tile_identifier const& other)const { return data_ == other.data_; }
  bool operator==(object_identifier const& other)const {
    if (object_identifier const* foo = get_object_identifier()) { return *foo == other; }
    else return false;
  }
  bool operator==(tile_location const& other)const {
    if (tile_location const* foo = get_tile_location()) { return *foo == other; }
    else return false;
  }
  bool operator!=(object_identifier const& other)const { return !(*this == other); }
  bool operator!=(tile_location const& other)const { return !(*this == other); }

  bool operator<(object_or_tile_identifier const& other)const { return data_ < other.data_; }

  friend inline std::ostream& operator<<(std::ostream& os, object_or_tile_identifier const& id) {
    struct ostream_visitor : public boost::static_visitor<void>
    {
      ostream_visitor(std::ostream& os) : os(os) {}
      std::ostream& os;
      
      void operator()(tile_location const& i) const { os << i; }
      void operator()(object_identifier i) const { os << i; }
    };
    boost::apply_visitor( ostream_visitor(os), id.data_ );
    return os;
  }
private:
  boost::variant<tile_location, object_identifier> data_;
};

namespace std {
  template<> struct hash<object_or_tile_identifier> {
    inline size_t operator()(object_or_tile_identifier const& id) const {
      return hash_value(id);
    }
  };
}
class object {
public:
  virtual shape get_initial_personal_space_shape()const = 0;
  virtual shape get_initial_detail_shape()const = 0;
  /*
private:
  virtual void this_virtual_function_makes_this_class_virtual_which_we_need_in_order_to_dynamic_cast_it(){};*/
};

class mobile_object : virtual public object {
public:
  //virtual void move_due_to_velocity() = 0;

  mobile_object():velocity(0,0,0){}
  mobile_object(vector3<fine_scalar> velocity):velocity(velocity){}
  vector3<fine_scalar> velocity;
};

class tile_aligned_object : virtual public object {
public:
  
};

class autonomous_object : virtual public object {
public:
  virtual void update(world& w, object_identifier my_id) = 0;
};


typedef bbox_collision_detector<object_or_tile_identifier, 64, 3> world_collision_detector;


namespace the_decomposition_of_the_world_into_blocks_impl {
  const int worldblock_dimension_exp = 4;
  typedef int worldblock_dimension_type;
  const worldblock_dimension_type worldblock_dimension = (1 << worldblock_dimension_exp);

  class worldblock {
  public:
    worldblock():neighbors_(nullptr),current_tile_realization_(COMPLETELY_IMAGINARY),is_busy_realizing_(false),w_(nullptr){}
    void construct(world* w, vector3<tile_coordinate> global_position) { w_ = w; global_position_ = global_position; }
    bool is_constructed() const { return w_ != nullptr; }
    worldblock& ensure_realization(level_of_tile_realization_needed realineeded) {
      // This function gets called to do nothing a LOT more than it gets called to actually do something;
      // return ASAP if we don't have to do anything.
      if (realineeded > current_tile_realization_) { this->ensure_realization_impl(realineeded); }
      return *this;
    }
    // ensure_realization_impl is a lot of code, with nontrivial duration, and should not be inlined.
    worldblock& ensure_realization_impl(level_of_tile_realization_needed realineeded);
  
    // Prefer to use tile_location::stuff_at().
    inline tile& get_tile(vector3<tile_coordinate> global_coords) {
      return tiles_[get_primitive_int(global_coords.x - global_position_.x)]
                   [get_primitive_int(global_coords.y - global_position_.y)]
                   [get_primitive_int(global_coords.z - global_position_.z)];
    }
  
    // Only to be used in tile_location::get_neighbor:
    template<cardinal_direction Dir> bool crossed_boundary(tile_coordinate new_coord);
    template<cardinal_direction Dir> tile_location get_neighboring_loc(vector3<tile_coordinate> const& old_coords, level_of_tile_realization_needed realineeded);

    template<cardinal_direction Dir> worldblock& ensure_neighbor_realization(level_of_tile_realization_needed realineeded);
    template<cardinal_direction Dir> tile_location get_loc_across_boundary(vector3<tile_coordinate> const& new_coords, level_of_tile_realization_needed realineeded);
    tile_location get_loc_guaranteed_to_be_in_this_block(vector3<tile_coordinate> coords);

    // an implementation detail of ensure_realization
    template<cardinal_direction Dir> void check_local_caches_cross_worldblock_neighbor(tile_coordinate x, tile_coordinate y, tile_coordinate z);
    tile_contents estimate_most_frequent_tile_contents_type()const;
  private:
    value_for_each_cardinal_direction<worldblock*> neighbors_;
    vector3<tile_coordinate> global_position_; // the lowest x, y, and z among elements in this worldblock
    level_of_tile_realization_needed current_tile_realization_;
    bool is_busy_realizing_;
    world* w_;
    std::array<std::array<std::array<tile, worldblock_dimension>, worldblock_dimension>, worldblock_dimension> tiles_;
  };
}

class world_building_gun;
typedef std::function<void (world_building_gun, tile_bounding_box)> worldgen_function_t;


typedef unordered_map<object_identifier, shape> object_shapes_t;
template<typename ObjectSubtype>
struct objects_map {
  typedef unordered_map<object_identifier, shared_ptr<ObjectSubtype>> type;
};







namespace tile_physics_impl {
struct state_t;
class tile_physics_state_t {
public:
  tile_physics_state_t(world& w);
  tile_physics_state_t(tile_physics_state_t const& other);
  tile_physics_state_t& operator=(tile_physics_state_t const& other);
  ~tile_physics_state_t() noexcept;
private:
  boost::scoped_ptr<state_t> state_;
  friend state_t& get_state(tile_physics_state_t& s);
};
inline state_t& get_state(tile_physics_state_t& s) {
  return *s.state_;
}
} // end namespace tile_physics_impl
using tile_physics_impl::tile_physics_state_t;









class world {
public:
  world(worldgen_function_t f);
  
  // If worlds being copiable is ever needed (a world is totally copiable in principle),
  // the error will remind us to make sure to implement the copy-constructor correctly
  // (e.g. without sharing any data via pointers by accident).
  world(world const& other) = delete;
  world& operator=(world const& other) = delete;
  
  void update_moving_objects();
  void update_fluids();

  // "Game time" attempts to match real world seconds, but it might
  // get behind due to slow CPUs or ahead due to someone fast-forwarding
  // (if that were a thing)
  // (and currently, also can get ahead because we don't cap the frame-rate.)
  time_unit game_time_elapsed()const { return current_game_time_; }

  void update(input_representation::input_news_t const& input_news);

  // TODO come up with a good way of farming this information out to robots once
  // we know how we want that to work, and a good way of representing multiple
  // players' input.
  input_representation::input_news_t const& input_news() const { return input_news_; }

  large_fast_noncrypto_rng& get_rng() { return rng_; }
  
  // I *think* this pointer is valid as long as the shared_ptr exists
  shared_ptr<object>* get_object(object_identifier id) { return find_as_pointer(objects_, id); }
  /*boost::iterator_range<mobile_objects_map<mobile_object>::iterator> mobile_objects_range() {
    return boost::make_iterator_range(mobile_objects.begin(), mobile_objects.end());
  }*/
  boost::iterator_range<objects_map<mobile_object>::type::iterator> moving_objects_range() {
    return boost::make_iterator_range(moving_objects_.begin(), moving_objects_.end());
  }
  
  tile_location make_tile_location(vector3<tile_coordinate> const& coords, level_of_tile_realization_needed realineeded);
  
  void collect_things_exposed_to_collision_intersecting(vector<object_or_tile_identifier>& results, bounding_box const& bounds) {
    ensure_realization_of_space_(convert_to_smallest_superset_at_tile_resolution(bounds), FULL_REALIZATION);
    things_exposed_to_collision_.get_objects_overlapping(results, bounds);
  }
  void collect_things_exposed_to_collision_intersecting(vector<object_or_tile_identifier>& results, tile_bounding_box const& bounds) {
    ensure_realization_of_space_(bounds, FULL_REALIZATION);
    things_exposed_to_collision_.get_objects_overlapping(results, convert_to_fine_units(bounds));
  }
  
  // old_substance_type doesn't actually do anything except give an assertion.
  // However, it's imperative that your code not accidentally overwrite a cool
  // type of substance that you weren't considering, so the assertion is built
  // into the function call to make sure that you use it.
  void replace_substance(
     tile_location const& loc,
     tile_contents old_substance_type,
     tile_contents new_substance_type);
  
  object_identifier try_create_object(shared_ptr<object> obj) {
    // TODO: fail (and return NO_OBJECT) if there's something in the way
    object_identifier id = next_object_identifier_++;
    objects_.insert(make_pair(id, obj));
    bounding_box b; // TODO: in mobile_objects.cpp, include detail_shape in at least the final box left in the ztree
    object_personal_space_shapes_[id] = obj->get_initial_personal_space_shape();
    b.combine_with(object_personal_space_shapes_[id].bounds());
    object_detail_shapes_[id] = obj->get_initial_detail_shape();
    b.combine_with(object_detail_shapes_[id].bounds());
    things_exposed_to_collision_.insert(id, b);
    if(shared_ptr<mobile_object> m = boost::dynamic_pointer_cast<mobile_object>(obj)) {
      moving_objects_.insert(make_pair(id, m));
    }
    // TODO: don't do this if you're in the middle of updating autonomous objects
    if(shared_ptr<autonomous_object> m = boost::dynamic_pointer_cast<autonomous_object>(obj)) {
      autonomously_active_objects_.insert(make_pair(id, m));
    }
    return id;
  }
  
  // If objects overlap with the new position, returns their IDs. If not, changes the shape and returns an empty set.
  //unordered_set<object_or_tile_identifier> try_to_change_personal_space_shape(object_identifier id, shape const& new_shape);
  // Objects can't fail to change their detail shape, but it may cause effects (like blocking a laser beam)
  //void change_detail_shape(object_identifier id, shape const& new_shape);
  
  void add_laser_sfx(vector3<fine_scalar> laser_source, vector3<fine_scalar> laser_delta) {
    laser_sfxes.push_back(make_pair(laser_source, laser_delta));
  }
  std::vector<std::pair<vector3<fine_scalar>, vector3<fine_scalar>>> laser_sfxes;

  // TODO: Either
  // 1) split this function into a personal_space version and a detail version, or
  // 2) make it explicit that it's the bounding box including both (that's what it is currently, or at least what it SHOULD be currently), or
  // 3) remove it / come up with something different to replace it with
  bounding_box get_bounding_box_of_object_or_tile(object_or_tile_identifier id)const;
  shape get_personal_space_shape_of_object_or_tile(object_or_tile_identifier id)const;
  shape get_detail_shape_of_object_or_tile(object_or_tile_identifier id)const;
  
  objects_map<object>::type const& get_objects()const { return objects_; }
  object_shapes_t const& get_object_personal_space_shapes()const { return object_personal_space_shapes_; }
  object_shapes_t const& get_object_detail_shapes()const { return object_detail_shapes_; }

  tile_physics_state_t& tile_physics() { return tile_physics_state_; }
  world_collision_detector const& get_things_exposed_to_collision()const { return things_exposed_to_collision_; }

  
private:
  friend class world_building_gun;
  friend class the_decomposition_of_the_world_into_blocks_impl::worldblock; // No harm in doing this, because worldblock is by definition already hacky.
  
  time_unit current_game_time_;
  
  // This map uses the same coordinates as worldblock::global_position - i.e. worldblocks' coordinates are multiples of worldblock_dimension, and it is an error to give a coordinate that's not.
  unordered_map<vector3<tile_coordinate>, the_decomposition_of_the_world_into_blocks_impl::worldblock> blocks_; 

  tile_physics_state_t tile_physics_state_;
  
  objects_map<object>::type objects_;
  objects_map<mobile_object>::type moving_objects_;
  objects_map<autonomous_object>::type autonomously_active_objects_;
  
  object_identifier next_object_identifier_;
  vector<shared_ptr<object>> objects_to_add_;
  object_shapes_t object_personal_space_shapes_;
  object_shapes_t object_detail_shapes_;
  
  // This currently means all mobile objects,
  // and all non-interior, non-air tiles.
  world_collision_detector things_exposed_to_collision_;
  
  // Worldgen functions TODO describe them
  worldgen_function_t worldgen_function_;

  input_representation::input_news_t input_news_;

  // RNG, default-initialized for now
  large_fast_noncrypto_rng rng_;
  
  
  the_decomposition_of_the_world_into_blocks_impl::worldblock* ensure_realization_of_and_get_worldblock_(vector3<tile_coordinate> position, level_of_tile_realization_needed realineeded);
  void ensure_realization_of_space_(tile_bounding_box space, level_of_tile_realization_needed realineeded);
  
  // Used only by world_building_gun
  void initialize_tile_contents_(tile_location const& loc, tile_contents contents);
  // Used only in the worldblock stuff
  void initialize_tile_local_caches_(tile_location const& loc);
  void initialize_tile_local_caches_relating_to_this_neighbor_(tile_location const& loc, tile neighbor);
  void initialize_tile_water_group_caches_(tile_location const& loc);
};


// some worldblock-related impls here for inlining purposes (speed)
inline tile const& tile_location::stuff_at()const { return wb_->get_tile(v_); }
template<cardinal_direction Dir> inline tile_location tile_location::get_neighbor(level_of_tile_realization_needed realineeded)const {
  return wb_->get_neighboring_loc<Dir>(v_, realineeded);
}
namespace the_decomposition_of_the_world_into_blocks_impl {
  template<> inline bool worldblock::crossed_boundary<xminus>(tile_coordinate new_coord) { return new_coord < global_position_.x; }
  template<> inline bool worldblock::crossed_boundary<yminus>(tile_coordinate new_coord) { return new_coord < global_position_.y; }
  template<> inline bool worldblock::crossed_boundary<zminus>(tile_coordinate new_coord) { return new_coord < global_position_.z; }
  template<> inline bool worldblock::crossed_boundary<xplus>(tile_coordinate new_coord) { return new_coord >= global_position_.x + worldblock_dimension; }
  template<> inline bool worldblock::crossed_boundary<yplus>(tile_coordinate new_coord) { return new_coord >= global_position_.y + worldblock_dimension; }
  template<> inline bool worldblock::crossed_boundary<zplus>(tile_coordinate new_coord) { return new_coord >= global_position_.z + worldblock_dimension; }

  template<cardinal_direction Dir> inline tile_location worldblock::get_neighboring_loc(vector3<tile_coordinate> const& old_coords, level_of_tile_realization_needed realineeded) {
    ensure_realization(realineeded);
    vector3<tile_coordinate> new_coords = old_coords; cdir_info<Dir>::add_to(new_coords);
    if (crossed_boundary<Dir>(new_coords[cdir_info<Dir>::dimension])) return tile_location(new_coords, &ensure_neighbor_realization<Dir>(realineeded));
    else return tile_location(new_coords, this);
  }

  template<cardinal_direction Dir> inline worldblock& worldblock::ensure_neighbor_realization(level_of_tile_realization_needed realineeded) {
    if (worldblock* neighbor = neighbors_[Dir]) {
      return neighbor->ensure_realization(realineeded);
    }
    else {
      return *(neighbors_[Dir] =
        w_->ensure_realization_of_and_get_worldblock_(
          global_position_ + vector3<worldblock_dimension_type>(cdir_info<Dir>::as_vector()) * worldblock_dimension,
          realineeded
        )
      );
    }
  }
}

#endif

