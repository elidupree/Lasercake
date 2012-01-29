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

using std::pair;
using std::make_pair;
using std::map;
using std::set;
using std::unordered_map;
using std::unordered_set;
using std::vector;
using std::array;
using boost::shared_ptr;


typedef int64_t fine_scalar; // Fine as opposed to coarse, that is.
typedef int32_t sub_tile_distance; // We can fit it within 32 bits, so we might as well do faster math

const fine_scalar tile_width = (fine_scalar(1) << 10);
const fine_scalar tile_height = (fine_scalar(1) << 10) / 5 + 1;
const vector3<fine_scalar> tile_size(tile_width, tile_width, tile_height);

const fine_scalar velocity_scale_factor = (fine_scalar(1) << 6);

const tile_coordinate world_center_coord = (tile_coordinate(1) << (8*sizeof(tile_coordinate) - 1));
const vector3<tile_coordinate> world_center_coords(world_center_coord, world_center_coord, world_center_coord);

const sub_tile_distance min_convincing_speed           = velocity_scale_factor * tile_width / 50;
const sub_tile_distance gravity_acceleration_magnitude = velocity_scale_factor * tile_width / 200;
const vector3<sub_tile_distance> gravity_acceleration(0, 0, -gravity_acceleration_magnitude); // in mini-units per frame squared
const sub_tile_distance friction_amount                = velocity_scale_factor * tile_width / 1800;

// TODO: Get some of these constants out of the header that everyone includes
const sub_tile_distance pressure_per_depth_in_tile_heights = gravity_acceleration_magnitude * tile_height * velocity_scale_factor;

// I believe this value makes it so that the terminal velocity of falling fluid is "half a vertical tile per frame".
const sub_tile_distance air_resistance_constant = tile_height * tile_height * velocity_scale_factor * velocity_scale_factor / gravity_acceleration_magnitude / 2;
const sub_tile_distance idle_progress_reduction_rate = 20 * velocity_scale_factor;

const vector3<sub_tile_distance> inactive_fluid_velocity(0, 0, -min_convincing_speed);

const fine_scalar max_object_speed_through_water = tile_width * velocity_scale_factor / 16;


inline vector3<tile_coordinate> get_containing_tile_coordinates(vector3<fine_scalar> v) {
  return vector3<tile_coordinate>(
    tile_coordinate(v.x / tile_width),
    tile_coordinate(v.y / tile_width),
    tile_coordinate(v.z / tile_height)
  );
}
inline fine_scalar lower_bound_in_fine_units(tile_coordinate c, int which_coordinate) {
  if (which_coordinate == 2) return c * tile_height;
  else                       return c * tile_width;
}
inline fine_scalar upper_bound_in_fine_units(tile_coordinate c, int which_coordinate) {
  if (which_coordinate == 2) return c * tile_height + (tile_height-1);
  else                       return c * tile_width + (tile_width-1);
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

typedef uint64_t object_identifier;
const object_identifier NO_OBJECT = 0;

struct object_or_tile_identifier {
  object_or_tile_identifier():data(NO_OBJECT){}
  object_or_tile_identifier(tile_location const& loc):data(loc){}
  object_or_tile_identifier(object_identifier id):data(id){}
  tile_location const* get_tile_location()const { return boost::get<tile_location>(&data); }
  object_identifier const* get_object_identifier()const { return boost::get<object_identifier>(&data); }
  size_t hash()const {
    struct hash_visitor : public boost::static_visitor<size_t>
    {
      size_t operator()(tile_location const& i) const {
        return std::hash<tile_location>()(i);
      }
    
      size_t operator()(object_identifier i) const {
        return std::hash<object_identifier>()(i);
      }
    };
    return boost::apply_visitor( hash_visitor(), data );
  }
  bool operator==(object_or_tile_identifier const& other)const { return data == other.data; }
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
private:
  boost::variant<tile_location, object_identifier> data;
};

namespace std {
  template<> struct hash<object_or_tile_identifier> {
    inline size_t operator()(object_or_tile_identifier const& id) const {
      return id.hash();
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
  mobile_object(vector3<fine_scalar>):velocity(velocity){}
  vector3<fine_scalar> velocity;
};

class tile_aligned_object : virtual public object {
public:
  
};

class autonomous_object : virtual public object {
public:
  virtual void update(world& w, object_identifier my_id) = 0;
};


class world_collision_detector {
private:
  typedef bbox_collision_detector<object_or_tile_identifier, 64, 3> internal_t;
  typedef internal_t::generalized_object_collection_handler igoch;
  typedef internal_t::bounding_box ibb;
  static ibb convert_bb(bounding_box const& bb) {
    caller_correct_if(bb.is_anywhere, "Trying to pass nowhere-bounds to bbox_collision_detector, which has no such concept");
    ibb b;
    b.min[0] = bb.min.x;
    b.min[1] = bb.min.y;
    b.min[2] = bb.min.z;
    b.size[0] = bb.max.x + 1 - bb.min.x;
    b.size[1] = bb.max.y + 1 - bb.min.y;
    b.size[2] = bb.max.z + 1 - bb.min.z;
    return b;
  }
  static bounding_box convert_bb(ibb const& b) {
    bounding_box bb;
    bb.min.x = b.min[0];
    bb.min.y = b.min[1];
    bb.min.z = b.min[2];
    bb.max.x = b.size[0] - 1 + b.min[0];
    bb.max.y = b.size[1] - 1 + b.min[1];
    bb.max.z = b.size[2] - 1 + b.min[2];
    bb.is_anywhere = true;
    return bb;
  }
public:
  world_collision_detector(){}
  
  void get_objects_overlapping(unordered_set<object_or_tile_identifier>& results, bounding_box const& bb)const {
    detector.get_objects_overlapping(results, convert_bb(bb));
  }
  
  void insert(object_or_tile_identifier id, bounding_box const& bb) {
    detector.insert(id, convert_bb(bb));
  }
  bool erase(object_or_tile_identifier id) {
    return detector.erase(id);
  }
  bool exists(object_or_tile_identifier id)const {
    return detector.exists(id);
  }
  // Returns bounding_box() (i.e. !is_anywhere) iff the id isn't in the detector.
  bounding_box find_bounding_box(object_or_tile_identifier id)const {
    const ibb* bb = detector.find_bounding_box(id);
    if(bb) return convert_bb(*bb);
    else return bounding_box();
  }
  
  class generalized_object_collection_handler {
  public:
    generalized_object_collection_handler():intermediary_(this){}
    virtual void handle_new_find(object_or_tile_identifier) {}
    virtual bool should_be_considered__static(bounding_box const&)const { return true; }
    virtual bool should_be_considered__dynamic(bounding_box const&)const { return true; }
    virtual bool bbox_ordering(bounding_box const& bb1, bounding_box const& bb2)const { return bb1.min < bb2.min; }
    virtual bool done()const { return false; }
    unordered_set<object_or_tile_identifier> const& get_found_objects()const { return intermediary_.get_found_objects(); }
  private:
    friend class world_collision_detector;
    struct impl_ : public igoch {
      world_collision_detector::generalized_object_collection_handler *outer;
      impl_(world_collision_detector::generalized_object_collection_handler *outer):outer(outer){}
      void handle_new_find(object_or_tile_identifier id) { outer->handle_new_find(id); }
      bool should_be_considered__static(ibb const& bb)const { return outer->should_be_considered__static(convert_bb(bb)); }
      bool should_be_considered__dynamic(ibb const& bb)const { return outer->should_be_considered__dynamic(convert_bb(bb)); }
      bool bbox_ordering(ibb const& bb1, ibb const& bb2)const { return outer->bbox_ordering(convert_bb(bb1), convert_bb(bb2)); }
      bool done()const { return outer->done(); }
    };
    impl_ intermediary_;
  };
  
  void get_objects_generalized(generalized_object_collection_handler *handler)const {
    detector.get_objects_generalized(&handler->intermediary_);
  }
  
private:
  internal_t detector;
};


namespace hacky_internals {
  const int worldblock_dimension_exp = 4;
  typedef int worldblock_dimension_type;
  const worldblock_dimension_type worldblock_dimension = (1 << worldblock_dimension_exp);

  class worldblock {
public:
    worldblock():neighbors(nullptr),w(nullptr),current_tile_realization(COMPLETELY_IMAGINARY),is_busy_realizing(false){}
    worldblock& ensure_realization(level_of_tile_realization_needed realineeded, world *w_ = nullptr, vector3<tile_coordinate> global_position_ = vector3<tile_coordinate>(0,0,0));
  
    // Only to be used in tile_location::stuff_at():
    inline tile& get_tile(vector3<tile_coordinate> global_coords) {
      return tiles[global_coords.x - global_position.x][global_coords.y - global_position.y][global_coords.z - global_position.z];
    }
  
    // Only to be used in tile_location::get_neighbor:
    template<cardinal_direction Dir> bool crossed_boundary(tile_coordinate new_coord);
    template<cardinal_direction Dir> tile_location get_neighboring_loc(vector3<tile_coordinate> const& old_coords, level_of_tile_realization_needed realineeded);
  
    template<cardinal_direction Dir> tile_location get_loc_across_boundary(vector3<tile_coordinate> const& new_coords, level_of_tile_realization_needed realineeded);
    tile_location get_loc_guaranteed_to_be_in_this_block(vector3<tile_coordinate> coords);
private:
    std::array<std::array<std::array<tile, worldblock_dimension>, worldblock_dimension>, worldblock_dimension> tiles;
    value_for_each_cardinal_direction<worldblock*> neighbors;
    vector3<tile_coordinate> global_position; // the lowest x, y, and z among elements in this worldblock
    world *w;
    level_of_tile_realization_needed current_tile_realization;
    bool is_busy_realizing;
  };
}
inline tile const& tile_location::stuff_at()const { return wb->get_tile(v); }
template<cardinal_direction Dir> tile_location tile_location::get_neighbor(level_of_tile_realization_needed realineeded)const {
  return wb->get_neighboring_loc<Dir>(v, realineeded);
}

class world_building_gun {
public:
  world_building_gun(world* w, tile_bounding_box bounds):w(w),bounds(bounds){}
  void operator()(tile_contents new_contents, vector3<tile_coordinate> locv);
private:
  world* w;
  tile_bounding_box bounds;
};

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
  world(worldgen_function_t f):tile_physics_state(*this),next_object_identifier(1),worldgen_function(f){}
  
  void update_moving_objects();
  void update_fluids();

  inline void update() {
    laser_sfxes.clear();
    update_fluids();
    for (auto& obj : autonomously_active_objects) obj.second->update(*this, obj.first);
    update_moving_objects();
  }
  
  // I *think* this pointer is valid as long as the shared_ptr exists
  shared_ptr<object>* get_object(object_identifier id) { return find_as_pointer(objects, id); }
  /*boost::iterator_range<mobile_objects_map<mobile_object>::iterator> mobile_objects_range() {
    return boost::make_iterator_range(mobile_objects.begin(), mobile_objects.end());
  }*/
  boost::iterator_range<objects_map<mobile_object>::type::iterator> moving_objects_range() {
    return boost::make_iterator_range(moving_objects.begin(), moving_objects.end());
  }
  
  tile_location make_tile_location(vector3<tile_coordinate> const& coords, level_of_tile_realization_needed realineeded);
  
  void collect_things_exposed_to_collision_intersecting(unordered_set<object_or_tile_identifier>& results, bounding_box const& bounds) {
    ensure_realization_of_space(convert_to_smallest_superset_at_tile_resolution(bounds), FULL_REALIZATION);
    things_exposed_to_collision.get_objects_overlapping(results, bounds);
  }
  void collect_things_exposed_to_collision_intersecting(unordered_set<object_or_tile_identifier>& results, tile_bounding_box const& bounds) {
    ensure_realization_of_space(bounds, FULL_REALIZATION);
    things_exposed_to_collision.get_objects_overlapping(results, convert_to_fine_units(bounds));
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
    object_identifier id = next_object_identifier++;
    objects.insert(make_pair(id, obj));
    bounding_box b; // TODO: in mobile_objects.cpp, include detail_shape in at least the final box left in the ztree
    object_personal_space_shapes[id] = obj->get_initial_personal_space_shape();
    b.combine_with(object_personal_space_shapes[id].bounds());
    object_detail_shapes[id] = obj->get_initial_detail_shape();
    b.combine_with(object_detail_shapes[id].bounds());
    things_exposed_to_collision.insert(id, b);
    if(shared_ptr<mobile_object> m = boost::dynamic_pointer_cast<mobile_object>(obj)) {
      moving_objects.insert(make_pair(id, m));
    }
    // TODO: don't do this if you're in the middle of updating autonomous objects
    if(shared_ptr<autonomous_object> m = boost::dynamic_pointer_cast<autonomous_object>(obj)) {
      autonomously_active_objects.insert(make_pair(id, m));
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
  
  objects_map<object>::type const& get_objects()const { return objects; }
  object_shapes_t const& get_object_personal_space_shapes()const { return object_personal_space_shapes; }
  object_shapes_t const& get_object_detail_shapes()const { return object_detail_shapes; }

  tile_physics_state_t& tile_physics() { return tile_physics_state; }
  world_collision_detector const& get_things_exposed_to_collision()const { return things_exposed_to_collision; }

  
private:
  friend class world_building_gun;
  friend class hacky_internals::worldblock; // No harm in doing this, because worldblock is by definition already hacky.
  
  // This map uses the same coordinates as worldblock::global_position - i.e. worldblocks' coordinates are multiples of worldblock_dimension, and it is an error to give a coordinate that's not.
  unordered_map<vector3<tile_coordinate>, hacky_internals::worldblock> blocks; 

  tile_physics_state_t tile_physics_state;
  
  objects_map<object>::type objects;
  objects_map<mobile_object>::type moving_objects;
  objects_map<autonomous_object>::type autonomously_active_objects;
  
  object_identifier next_object_identifier;
  vector<shared_ptr<object>> objects_to_add;
  object_shapes_t object_personal_space_shapes;
  object_shapes_t object_detail_shapes;
  
  // This currently means all mobile objects, all water, and surface rock tiles. TODO I haven't actually implemented restricting to sufface rock yet
  world_collision_detector things_exposed_to_collision;
  
  // Worldgen functions TODO describe them
  worldgen_function_t worldgen_function;
  
  
  hacky_internals::worldblock* ensure_realization_of_and_get_worldblock(vector3<tile_coordinate> position, level_of_tile_realization_needed realineeded);
  void ensure_realization_of_space(tile_bounding_box space, level_of_tile_realization_needed realineeded);
  
  // Used only by world_building_gun
  void initialize_tile_contents(tile_location const& loc, tile_contents contents);
  // Used only in the worldblock stuff
  void initialize_tile_local_caches(tile_location const& loc);
  void initialize_tile_water_group_caches(tile_location const& loc);
};

#endif

