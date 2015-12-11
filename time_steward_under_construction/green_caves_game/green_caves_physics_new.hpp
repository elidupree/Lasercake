/*

    Copyright Eli Dupree and Isaac Dupree, 2014

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LASERCAKE_GREEN_CAVES_PHYSICS_NEW_HPP__
#define LASERCAKE_GREEN_CAVES_PHYSICS_NEW_HPP__

#include "../time_steward.hpp"
#include "cave_generator.hpp"

typedef time_steward_system:: default_time_traits:: time_type time_type;
const time_type never = time_steward_system:: default_time_traits:: never;
typedef time_steward_system:: entity_id entity_id;
typedef time_steward_system:: entity_id entity_ID;

typedef int64_t space_coordinate;
const num_coordinates_type num_dimensions = 2;

const int tile_size_shift = 20;
const space_coordinate tile_size = space_coordinate (1) << tile_size_shift;
const time_type second_time = time_type (1) << 10;

struct wall_destroyed {};
struct player_radius {};
struct trajectory {
  time_type start_time;
  space_vector start_location;
  space_vector velocity;
};

template <typename Accessor>
cave_generator:: tile_info get_tile (Accessor accessor, tile_vector tile) {
  auto result = accessor.get_constant <cave_generator> ().get (tile);
  if (result.wall && accessor -> is_set <wall_destroyed> (result.ID)) {
    result.wall = false;
  }
  return result;
}
template <typename Accessor>
entity_ID tile_ID (Accessor accessor, tile_vector tile) {
  return accessor.get_constant <cave_generator> ().get (tile) .ID;
}
template <typename Accessor>
bool is_shot (Accessor accessor, entity_ID ID) {
  return ! accessor.is_set <player_radius> (ID);
}


template <typename Accessor>
bool shot_hits (Accessor accessor, entity_ID ID, tile_vector tile) {
    const auto new_tile_info = get_tile (accessor, tile);
    if (new_tile_info.wall) {
      accessor.set <wall_destroyed> (new_tile_info.ID);
      accessor.erase <trajectory> (ID);
      accessor.erase <reference_tile> (ID);
      return true;
    }
    return false;  
}

template <typename Accessor>
void tile_change_predictor (Accessor accessor, entity_ID ID) {
  const auto trajectory = accessor.get <trajectory> (ID);
  if (! accessor.is_set <reference_tile> (ID)) {
    accessor.predict (accessor.immediately (), [ID] (Accessor accessor) {
      const auto trajectory = accessor.get <trajectory> (ID);
      accessor.set <reference_tile> (ID, tile_vector (tile_to_space_min (trajectory.start_location + trajectory.velocity*(accessor.now () - trajectory.start_time))));
    });
    return;
  }
  
  const auto tile = accessor.get <reference_tile> (ID);
  space_coordinate threshold_X = (trajectory.velocity (0) >0? tile_to_space_max (tile (0)): tile_to_space_min (tile (0))) - trajectory.start_location (0);
  space_coordinate threshold_Y = trajectory.velocity (1) >0? tile_to_space_max (tile (1)): tile_to_space_min (tile (1))) - trajectory.start_location (1);
  space_coordinate compare;
  if (trajectory.velocity (0) == 0) {compare = 1;}
  else if (trajectory.velocity (1) == 0) {compare = -1;}
  else {
    //compare = abs (velocity (1)/velocity (0))-abs (threshold_Y/threshold_X)
    compare = velocity (1)*threshold_X - velocity (0)*threshold_Y;
    compare *= sign (velocity (0)*threshold_X);
  }
  
  time_type threshold_time;
  if (compare >= 0) {
    threshold_time = trajectory.start_time + divide (threshold_Y, velocity (1), rounding_strategy <round_up, negative_is_forbidden> ());
  }
  else {
    threshold_time = trajectory.start_time + divide (threshold_X, velocity (0), rounding_strategy <round_up, negative_is_forbidden> ());
  }
  
  accessor.predict (threshold_time, [ID, tile, compare] (Accessor accessor) {
    bool shot = is_shot (accessor, ID);
    if (shot) {
      accessor.set_group_membership <tile_shots> (tile_ID (accessor, tile), ID, false);
    }
    if (compare == 0 && shot) {
      uint64_t which_first = accessor.random_bits (1);
      for (uint64_t neighbor = 0; neighbor <2;++neighbor) {
        tile_vector neighbor_tile = tile;
        neighbor_tile [neighbor ^ which_first] += sign (trajectory.velocity (neighbor ^ which_first));
        if (shot_hits (accessor, ID, neighbor_tile)) {return;}
      }
    }
    tile_vector new_tile = tile;
    if (compare >= 0) {new_tile [1] += sign (trajectory.velocity (1));}
    if (compare <= 0) {new_tile [0] += sign (trajectory.velocity (0));}
    if (shot) {
      if (shot_hits (accessor, ID, new_tile)) {return;}
      accessor.set_group_membership <tile_shots> (tile_ID (new_tile), ID, true);
    }
    accessor.set <reference_tile> (ID, new_tile);
  });
}


template <typename Accessor>
void player_hits_walls_predictor (Accessor accessor, entity_ID ID) {
  const auto center = accessor.get <trajectory> (ID);
  const auto player_tile = accessor.get <reference_tile> (ID);
  const auto radius = accessor.get <player_radius> (ID);
  tile_vector minima;
  tile_vector maxima;
  for (num_coordinates_type dimension = 0; dimension <2;++dimension) {
    minima [dimension] = space_to_tile_min (tile_to_space_min (player_tile (dimension)) - radius);
    maxima [dimension] = space_to_tile_max (tile_to_space_max (player_tile (dimension)) + radius);
  }
  tile_vector tile;
  tile_vector best_tile;
  time_type best_time = never;
  for (tile [0] = minima (0); tile [0] <= maxima (0);++tile [0]) {
    for (tile [1] = minima (1); tile [1] <= maxima (1);++tile [1]) {
      if (!get_tile (accessor, tile).wall) continue;
      time_type when = never;
      when = time_min (when, when_trajectory_intersects_circle (
        center, tile_to_space_min (tile (0)), tile_to_space_min (tile (1))));
      when = time_min (when, when_trajectory_intersects_circle (
        center, tile_to_space_min (tile (0)), tile_to_space_max (tile (1))));
      when = time_min (when, when_trajectory_intersects_circle (
        center, tile_to_space_max (tile (0)), tile_to_space_min (tile (1))));
      when = time_min (when, when_trajectory_intersects_circle (
        center, tile_to_space_max (tile (0)), tile_to_space_max (tile (1))));
      when = time_min (when, when_trajectory_intersects_rectangle (center,
        tile_to_space_min (tile (0)) - radius, tile_to_space_min (tile (1)),
        tile_to_space_max (tile (0)) + radius, tile_to_space_max (tile (1))));
      when = time_min (when, when_trajectory_intersects_rectangle (center,
        tile_to_space_min (tile (0)), tile_to_space_min (tile (1)) - radius,
        tile_to_space_max (tile (0)), tile_to_space_max (tile (1)) + radius));
      if (when != never) {
        if (best_time == never || when <best_time) {
          //note: the when == best_time case creates a bias towards
          //colliding with top-left walls first. However, I believe
          //that that doesn't make any difference.
          best_time = when;
          best_tile = tile;
        }
      }
    }
  }
  if (best_time != never) {
    accessor.predict (best_time - 1, [ID, best_tile] (Accessor accessor) {
      auto const & tile = best_tile;
      auto & center = accessor.get_mutable <trajectory> (ID);
      const auto radius = accessor.get <player_radius] (ID);
      center.start_location += center.velocity*(accessor.now () - center.start_time);
      center.start_time = accessor.now ();
      bool out_of_bounds_0 =
        (center.start_location (0) + radius <tile_to_space_min (tile (0))) ||
        (center.start_location (0) - radius >tile_to_space_max (tile (0)));
      bool out_of_bounds_1 =
        (center.start_location (1) + radius <tile_to_space_min (tile (1))) ||
        (center.start_location (1) - radius >tile_to_space_max (tile (1)));
      if (out_of_bounds_0 && ! out_of_bounds_1) {
        center.velocity [0] = 0;
      }
      else if (out_of_bounds_1 && ! out_of_bounds_0) {
        center.velocity [1] = 0;
      }
      else {
        space_coordinate mid_x = (tile_to_space_min(tile(0)) + tile_to_space_max(tile(0))) / 2;
        space_coordinate mid_y = (tile_to_space_min(tile(1)) + tile_to_space_max(tile(1))) / 2;
        space_vector corner ((center.start_location (0) < mid_x) ? tile_to_space_min(tile(0)) : tile_to_space_max(tile(0)), (center.start_location (1) < mid_y) ? tile_to_space_min(tile(1)) : tile_to_space_max(tile(1)));
        space_vector delta = center.start_location - corner;
        
      }
    });
  }
}

namespace implementation {
using time_steward_system:: field;

typedef time_steward_system:: physics_list <
  field <wall_destroyed>,
  field <trajectory>,
  field <reference_tile, tile_vector>,
  field <player_radius, space_coordinate>,
  predictor <trajectory, tile_change_predictor>,
  predictor <player_radius, player_hits_walls_predictor>
> green_caves_physics;
}
typedef implementation:: green_caves_physics green_caves_physics;

#endif

