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

template <typename Accessor>
cave_generator:: tile_info get_tile (Accessor accessor, tile_vector tile) {
  auto result = accessor.get_constant <cave_generator> ().get (tile);
  if (result.wall && accessor -> get <wall_destroyed> (result.ID)) {
    result.wall = false;
  }
  return result;
}
template <typename Accessor>
entity_ID tile_ID (Accessor accessor, tile_vector tile) {
  return accessor.get_constant <cave_generator> ().get (tile) .ID;
}

template <typename Accessor>
void shot_motion_predictor (Accessor accessor, entity_ID ID) {
  const auto trajectory = accessor.get <shot_trajectory> (ID);
  const auto tile = accessor.get <shot_tile> (ID);
  time_type best_time = never;
  tile_vector new_tile;
  for (num_coordinates_type dimension = 0; dimension <num_dimensions;++dimension) {
    time_type when = never;
    tile_vector where = tile;
    if (trajectory.velocity (dimension) >0) {
      ++where [dimension];
      when = when_nonnegative (something involving tile_to_space_max (tile (dimension)));
      assert (when != never);
    }
    if (trajectory.velocity (dimension) <0) {
      -- where [dimension];
      when = when_nonpositive (something involving tile_to_space_min (tile (dimension)));
      assert (when != never);
    }
    if (when != never && (best_time == never || best_time >when (
        best_time >= when && accessor.random_bits (1)))) {
      best_time = when;
      new_tile = where;
    }
  }
  assert (best_time != never);
  accessor.predict (best_time, [ID, new_tile] (Accessor accessor) {
    accessor.set_group_membership <tile_shots> (tile_ID (accessor, tile), ID, false);
    const auto new_tile_info = get_tile (accessor, new_tile);
    if (new_tile_info.wall) {
      accessor.set <wall_destroyed> (new_tile_info.ID, true);
      accessor.erase <shot_trajectory> (ID);
    }
    else {
      accessor.set <shot_tile> (ID, new_tile);
      accessor.set_group_membership <tile_shots> (new_tile_info .ID, ID, true);
    }
  });
}


namespace implementation {
using time_steward_system:: field;
typedef time_steward_system:: physics_list <
  field <wall_destroyed, bool, false>,
  
> green_caves_physics;
}
typedef implementation:: green_caves_physics green_caves_physics;

#endif

