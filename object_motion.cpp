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

#include <queue>
#include "world.hpp"

namespace /*anonymous*/ {

typedef lasercake_int<int64_t>::type time_int_type;
typedef non_normalized_rational<time_int_type> time_type;
typedef faux_optional<time_type> optional_time;

vector3<fine_scalar> movement_delta_from_start_to(vector3<fine_scalar> const& velocity, time_type end_time) {
  // Always round 'up'.
  // This is the simplest way to play nicely with the polyhedron sweep code.
  // It has the downside that it makes everything move, on average, 0.5 fine scalar units faster than it should
  // in each dimension of its movement. 
  /*return vector3<fine_scalar>(
    sign(velocity.x) * (
      ((std::abs(velocity.x) * end_time.numerator) + (end_time.denominator * velocity_scale_factor) - 1)
      / (                                             end_time.denominator * velocity_scale_factor)
    ),
    sign(velocity.y) * (
      ((std::abs(velocity.y) * end_time.numerator) + (end_time.denominator * velocity_scale_factor) - 1)
      / (                                             end_time.denominator * velocity_scale_factor)
    ),
    sign(velocity.z) * (
      ((std::abs(velocity.z) * end_time.numerator) + (end_time.denominator * velocity_scale_factor) - 1)
      / (                                             end_time.denominator * velocity_scale_factor)
    )
  );*/
  
  // Round to nearest:
  return vector3<fine_scalar>(
    divide_rounding_towards_zero(
      ((2 * velocity.x * end_time.numerator) + (end_time.denominator * velocity_scale_factor)),
      ( 2 *                                     end_time.denominator * velocity_scale_factor )
    ),
    divide_rounding_towards_zero(
      ((2 * velocity.y * end_time.numerator) + (end_time.denominator * velocity_scale_factor)),
      ( 2 *                                     end_time.denominator * velocity_scale_factor )
    ),
    divide_rounding_towards_zero(
      ((2 * velocity.z * end_time.numerator) + (end_time.denominator * velocity_scale_factor)),
      ( 2 *                                     end_time.denominator * velocity_scale_factor )
    )
  );
}

vector3<fine_scalar> movement_delta_rounding_down(vector3<fine_scalar> const& velocity, time_type end_time) {
  return vector3<fine_scalar>(
    divide_rounding_towards_zero(
      (velocity.x * end_time.numerator),
      (end_time.denominator * velocity_scale_factor)
    ),
    divide_rounding_towards_zero(
      (velocity.y * end_time.numerator),
      (end_time.denominator * velocity_scale_factor)
    ),
    divide_rounding_towards_zero(
      (velocity.z * end_time.numerator),
      (end_time.denominator * velocity_scale_factor)
    )
  );
}

vector3<fine_scalar> movement_delta_intermediate(vector3<fine_scalar> const& velocity, time_type min, time_type max) {
  return movement_delta_from_start_to(velocity, max) - movement_delta_from_start_to(velocity, min);
}

bool sweep_vs_shape (convex_polyhedron const& sweep, shape const* s2) {
    for (convex_polyhedron const& p2 : s2->get_polyhedra()) {
      if (!find_excluding_planes(sweep, p2)) {
        return true;
      }
    }
    for (bounding_box const& bb : s2->get_boxes()) {
      if (!find_excluding_planes(sweep, convex_polyhedron(bb))) {
        return true;
      }
    }
        return false;

}

// You can round up by up to 1.
// Something you're facing head-on can round up in the other direction by up to 1.
// 
const fine_scalar max_error_dist = 2;

optional_time get_first_moment_of_intersection_(shape const* s1, shape const* s2, vector3<fine_scalar> s1_velocity, vector3<fine_scalar> s2_velocity, time_type s1_last_time_updated, time_type s2_last_time_updated, vector3<fine_scalar> max_error, time_int_type which_step, time_int_type inverse_step_size) {
  //std::cerr << "Argh1: " << s1_last_time_updated << ", " << s2_last_time_updated << ", " << which_step <<  ", " << inverse_step_size << "\n";
  time_type step_begin(which_step, inverse_step_size);
  time_type step_end(which_step+1, inverse_step_size);
  //std::cerr << step_begin << step_end << "\n";
  if ((step_end < s1_last_time_updated) || (step_end < s2_last_time_updated)) return boost::none;
  
  // I'm not sure exactly what the step-ends should be; this is relatively conservative (because the rounding error will get them covered anyway).
  const vector3<fine_scalar> s1_delta_from_stored_location_to_step_begin = movement_delta_from_start_to(s1_velocity, step_begin - s1_last_time_updated);
  const vector3<fine_scalar> s2_delta_from_stored_location_to_step_begin = movement_delta_from_start_to(s2_velocity, step_begin - s2_last_time_updated);
  const vector3<fine_scalar> s1_delta_from_stored_location_to_step_end = movement_delta_from_start_to(s1_velocity, step_end - s1_last_time_updated);
  const vector3<fine_scalar> s2_delta_from_stored_location_to_step_end = movement_delta_from_start_to(s2_velocity, step_end - s2_last_time_updated);
    
  vector3<fine_scalar> relative_delta_to_step_begin =
    s1_delta_from_stored_location_to_step_begin
    - s2_delta_from_stored_location_to_step_begin;
  vector3<fine_scalar> relative_delta_to_step_end =
    s1_delta_from_stored_location_to_step_end
    - s2_delta_from_stored_location_to_step_end;
  
  vector3<fine_scalar> local_error = max_error;
  const vector3<fine_scalar> movement_this_step = relative_delta_to_step_end - relative_delta_to_step_begin;
  
  for (int dim = 0; dim < num_dimensions; ++dim) {
    fine_scalar fail = 
      (std::abs(s1_delta_from_stored_location_to_step_begin(dim)) >= 1) +
      (std::abs(s2_delta_from_stored_location_to_step_begin(dim)) >= 1);
    relative_delta_to_step_begin[dim] -= sign(max_error(dim)) * fail;
    local_error[dim] += sign(max_error(dim)) * fail * 2;
  }
  //std::cerr << "Argh2: " << which_step << ", " << inverse_step_size << ", " << s1_velocity << ", " << s2_velocity << ", " << relative_delta_to_step_begin << ", " << relative_delta_to_step_end << ", " << movement_this_step << " ERROR: " << max_error << "\n";
  bool intersects = false;
  bounding_box sweep_bounds = s1->bounds();
  sweep_bounds.translate(relative_delta_to_step_begin);
  bounding_box end_bounds(sweep_bounds);
  end_bounds.translate(movement_this_step+local_error);
  sweep_bounds.combine_with(end_bounds);
  if (sweep_bounds.overlaps(s2->bounds())) {
    // Hack, TODO fix: Only polyhedra collide properly!
    for (convex_polyhedron /*copy, not reference*/ p : s1->get_polyhedra()) {
      p.translate(relative_delta_to_step_begin);
      convex_polyhedron sweep(p, movement_this_step, local_error);
      if (sweep_vs_shape(sweep, s2)) {
        intersects = true;
        break;
      }
    }
  }
  
  // HACK: TODO FIX: Ignore results below the top level, because we're getting bad behavior when we don't.
  // Presumably this is the result of false-negatives in the sweep intersection code...
  if (/*(inverse_step_size > 1) ||*/ /*(((time_type(0) < s1_last_time_updated) || (time_type(0) < s2_last_time_updated)) && (s1_velocity != s2_velocity)) ||*/ intersects) {
  //std::cerr  << "argh3";
   // assert((s1_velocity != s2_velocity) || s1->intersects(*s2));
    // Because of the rounding error for the two shapes,
    // we may never get below a delta like (2,2,2).
    // Therefore, just stop at that minimum.
    //if (movement_this_step.magnitude_within_32_bits_is_less_than(tile_width / 30)) {
    if ((movement_this_step(X) <= 2) && (movement_this_step(X) >= -2) &&
        (movement_this_step(Y) <= 2) && (movement_this_step(Y) >= -2) &&
        (movement_this_step(Z) <= 2) && (movement_this_step(Z) >= -2)) {
      time_type result(which_step - 1, inverse_step_size);
  //std::cerr  << "argh4";
      if ((result < s1_last_time_updated) || (result < s2_last_time_updated)) {
        // We collide in a tiny moment that straddles the beginning of the sought time period.
        // Since we're supposed to assume they weren't initially colliding,
        // and they can't collide in the PAST,
        return std::max(s1_last_time_updated, s2_last_time_updated);
      }
      else return result;
    }
    else {
  //std::cerr  << "argh5";
      if (optional_time result =
               get_first_moment_of_intersection_(s1, s2, s1_velocity, s2_velocity, s1_last_time_updated, s2_last_time_updated, max_error,
                 (which_step * 2)    , inverse_step_size * 2)) {
        return result;
      }
      else {
        return get_first_moment_of_intersection_(s1, s2, s1_velocity, s2_velocity, s1_last_time_updated, s2_last_time_updated, max_error,
                 (which_step * 2) + 1, inverse_step_size * 2);
      }
    }
  }
  else {
  //std::cerr  << "argh6";
    shape seire(*s1);
    seire.translate(relative_delta_to_step_begin);
    assert(!seire.intersects(*s2));
    seire.translate(movement_this_step);
    assert(!seire.intersects(*s2));
    return boost::none;
  }
}

time_type round_time_downwards(time_type time, time_int_type max_meaningful_precision) {
  if (time.denominator < 0) {
    time.numerator = -time.numerator;
    time.denominator = -time.denominator;
  }
    //ilog2 is giving me weird errors I don't understand. TODO understand that and do this the right way
  /*if (farthest_plane_cross.denominator > max_meaningful_precision) {
    int shift_amount = ilog2(farthest_plane_cross.denominator) - ilog2(max_meaningful_precision);
    farthest_plane_cross.numerator >>= shift_amount;
    farthest_plane_cross.denominator = (farthest_plane_cross.denominator + (1 << shift_amount) - 1) >> shift_amount;
  }*/
  while (time.denominator > max_meaningful_precision) {
    int shift_amount = 2;
    time.numerator >>= shift_amount;
    time.denominator = (time.denominator + (1 << shift_amount) - 1) >> shift_amount;
  }
  return time;
}

time_type round_time_upwards(time_type time, time_int_type max_meaningful_precision) {
  if (time.denominator < 0) {
    time.numerator = -time.numerator;
    time.denominator = -time.denominator;
  }
    //ilog2 is giving me weird errors I don't understand. TODO understand that and do this the right way
  /*if (farthest_plane_cross.denominator > max_meaningful_precision) {
    int shift_amount = ilog2(farthest_plane_cross.denominator) - ilog2(max_meaningful_precision);
    farthest_plane_cross.numerator >>= shift_amount;
    farthest_plane_cross.denominator = (farthest_plane_cross.denominator + (1 << shift_amount) - 1) >> shift_amount;
  }*/
  while(time.denominator > max_meaningful_precision) {
    int shift_amount = 2;
    time.denominator >>= shift_amount;
    time.numerator = (time.numerator + (1 << shift_amount) - 1) >> shift_amount;
  }
  return time;
}

// This is essentially arbitrary - beyond a certain value it doesn't really matter, but there's no clear cutoff.
time_int_type max_meaningful_time_precision(vector3<fine_scalar> const& velocity) {
  return 16+std::max(std::max(std::abs(velocity(X)),std::abs(velocity(Y))),std::abs(velocity(Z))) * 16 / velocity_scale_factor;
}

time_type get_first_moment_of_all_overlapping(std::vector<plane_as_base_point_and_normal> const& excluding_planes, std::vector<vector3<fine_scalar>> const& other_vertices, vector3<fine_scalar> relative_velocity) {
  time_type farthest_plane_cross(0);
  for (auto const& p : excluding_planes) {
  //std::cerr << p.base_point << p.normal << relative_velocity << "\n";
    time_type closest_vertex_cross(1);
    const auto vel_dotprod = relative_velocity.dot<fine_scalar>(p.normal);
    if (vel_dotprod == 0) return time_type(0);
    for (auto const& v : other_vertices) {
      vector3<fine_scalar> worst_error(
        -sign(p.normal(X)),
        -sign(p.normal(Y)),
        -sign(p.normal(Z)));
      const auto dotprod = ((v - p.base_point) + worst_error).dot<fine_scalar>(p.normal);
      const time_type vertex_cross(dotprod * velocity_scale_factor, vel_dotprod);
        //std::cerr << vertex_cross << "\n";
      if (vertex_cross < closest_vertex_cross) {
        closest_vertex_cross = vertex_cross;
      }
    }
    if (closest_vertex_cross > farthest_plane_cross) {
      farthest_plane_cross = closest_vertex_cross;
    }
  }
        //std::cerr << farthest_plane_cross << "!\n";
  // Simplify...
  farthest_plane_cross = round_time_downwards(farthest_plane_cross, max_meaningful_time_precision(relative_velocity));
        //std::cerr << farthest_plane_cross << "!\n";
        return time_type(0);
  return farthest_plane_cross;
}

inline optional_time get_first_moment_of_intersection_old(shape const* s1, shape const* s2, vector3<fine_scalar> s1_velocity, vector3<fine_scalar> s2_velocity, time_type s1_last_time_updated, time_type s2_last_time_updated) {
  // Standardize: s1 is the one whose position is up-to-date.
  if (s2_last_time_updated > s1_last_time_updated) {
    return get_first_moment_of_intersection_old(s2, s1, s2_velocity, s1_velocity, s2_last_time_updated, s1_last_time_updated);
  }
  vector3<time_type> reciprocal_last_moments_before_rounding;
  std::array<int, num_dimensions> dimension_rounding_order = {{X,Y,Z}};
  for (int dim = 0; dim < num_dimensions; ++dim) {
    // obeys the rule 0.5 = time * (abs(velocity) / velocity_scale_factor)
    // thus time = 0.5 * (velocity_scale_factor / abs(velocity))
    
    //if (s2_last_time_updated > s1_last_time_updated) {
      if (s1_velocity(dim) == s2_velocity(dim)) {
        // If they're going at the same rate in the same direction, they never round relative to each other.
        reciprocal_last_moments_before_rounding[dim] = time_type(0, 1);
      }
      else {
        reciprocal_last_moments_before_rounding[dim] = time_type(2 * std::max(std::abs(s1_velocity(dim)), std::abs(s2_velocity(dim))), velocity_scale_factor);
      }
    /*}
    else {
      // hack - just do one step.
      // The worst-case scenario is a "false-"positive that causes a collision when it was
      // already a very close thing, whereupon the two objects will be updated to the same time for the next round of checking.
      reciprocal_last_moments_before_rounding[dim] = time_type(0, 1);
    }*/
  }
  
  // Sort them in ascending order BY TIME
  // i.e. descending order by reciprocal time
  // (i can haz cleaner code?)
  if (reciprocal_last_moments_before_rounding(X) < reciprocal_last_moments_before_rounding(Y)) {
    dimension_rounding_order[0] = Y; dimension_rounding_order[1] = X; }
  if (reciprocal_last_moments_before_rounding(dimension_rounding_order[1]) < reciprocal_last_moments_before_rounding(dimension_rounding_order[2])) {
    const auto temp = dimension_rounding_order[1];
    dimension_rounding_order[1] = dimension_rounding_order[2];
    dimension_rounding_order[2] = temp; }
  if (reciprocal_last_moments_before_rounding(dimension_rounding_order[0]) < reciprocal_last_moments_before_rounding(dimension_rounding_order[1])) {
    const auto temp = dimension_rounding_order[0];
    dimension_rounding_order[0] = dimension_rounding_order[1];
    dimension_rounding_order[1] = temp; }

  bool intersects = false;
  time_type force_min_intersect_duration;
  
  const vector3<fine_scalar> other_prior_displacement = movement_delta_from_start_to(s2_velocity, s1_last_time_updated - s2_last_time_updated);
  
  // Hack...
  shape s2_moved(*s2);
  s2_moved.translate(other_prior_displacement);
  if(s1->intersects(s2_moved)) return boost::none;

  time_type total_duration = time_type(1) - s1_last_time_updated;
  // Hack, TODO fix: Only polyhedra collide properly!
  for (convex_polyhedron /*copy, not reference*/ p : s1->get_polyhedra()) {
    p.translate(-other_prior_displacement);
    //std::cerr << "...\n";
    vector3<fine_scalar> delta_to_last_step_end(0,0,0);
    time_type last_duration(0);
    for (int i = 0; i < 3; ++i) {
      time_type duration = total_duration;
      if ((i < 2) && reciprocal_last_moments_before_rounding[i+1].numerator != 0) {
        int dim = dimension_rounding_order[i];
        auto recip_time = reciprocal_last_moments_before_rounding(dim);
        if (recip_time.numerator != 0) {
          duration = recip_time.reciprocal();
          if (duration > total_duration) duration = total_duration;
        }
      }
      const vector3<fine_scalar> delta_to_this_step_end = movement_delta_from_start_to(s1_velocity - s2_velocity, duration);
      //std::cerr << delta_to_this_step_end << dim << s1_velocity << s2_velocity << end_time << "\n";
      assert(!(duration < last_duration));
      assert((((delta_to_this_step_end(X) != 0) + (delta_to_this_step_end(Y) != 0) + (delta_to_this_step_end(Z) != 0)) <= (i+1)) || (duration == total_duration));
      if (delta_to_this_step_end != delta_to_last_step_end) {
        const vector3<fine_scalar> this_step_delta = delta_to_this_step_end - delta_to_last_step_end;
        convex_polyhedron sweep(p, this_step_delta, vector3<fine_scalar>(sign(delta_to_this_step_end(X)),sign(delta_to_this_step_end(Y)),sign(delta_to_this_step_end(Z))));
        
        const fine_scalar inverse_tinystep_size = std::max(max_meaningful_time_precision(s1_velocity), max_meaningful_time_precision(s1_velocity));
        
        if (sweep_vs_shape(sweep, s2)) {
          intersects = true;
          force_min_intersect_duration = last_duration;
          break;
        }
        else {
          for(time_type q(0/*(inverse_tinystep_size * s1_last_time_updated.numerator + s1_last_time_updated.denominator - 1) / s1_last_time_updated.denominator*/, inverse_tinystep_size); q < duration; ++q.numerator) {
            shape s1t(*s1); s1t.translate(movement_delta_from_start_to(s1_velocity, q));
            shape s2t(*s2); s2t.translate(movement_delta_from_start_to(s2_velocity, q + s1_last_time_updated - s2_last_time_updated));
            assert(!s1t.intersects(s2t));
          }
            shape s1t(*s1); s1t.translate(movement_delta_from_start_to(s1_velocity, duration));
            shape s2t(*s2); s2t.translate(movement_delta_from_start_to(s2_velocity, duration + s1_last_time_updated - s2_last_time_updated));
            assert(!s1t.intersects(s2t));
        }
        delta_to_last_step_end = delta_to_this_step_end;
        p.translate(this_step_delta);
      }
      last_duration = duration;
    }
  }
  
  if (intersects) {
    //std::cerr << "yo";
    time_type first_duration_of_collision = total_duration;
    // Hack, TODO fix: Only polyhedra collide properly!
    for (convex_polyhedron /*copy, not reference*/ p : s1->get_polyhedra()) {
      p.translate(-other_prior_displacement);
      for (convex_polyhedron const& p2 : s2->get_polyhedra()) {
        std::vector<plane_as_base_point_and_normal> excluding_planes;
        find_excluding_planes(p, p2, &excluding_planes);
        auto moment = get_first_moment_of_all_overlapping(excluding_planes, p2.vertices(), s1_velocity - s2_velocity);
        if (moment < first_duration_of_collision) {
          first_duration_of_collision = moment;
        }
      }
      for (bounding_box const& bb : s2->get_boxes()) {
        std::vector<plane_as_base_point_and_normal> excluding_planes;
        convex_polyhedron bbpoly(bb);
        find_excluding_planes(p, bbpoly, &excluding_planes);
        auto moment = get_first_moment_of_all_overlapping(excluding_planes, bbpoly.vertices(), s1_velocity - s2_velocity);
        if (moment < first_duration_of_collision) {
          first_duration_of_collision = moment;
        }
      }
    }
    time_type result;
    if (first_duration_of_collision < force_min_intersect_duration) {
      //std::cerr << "what: " << force_min_intersect_duration <<"," <<s1_last_time_updated << "\n";
      result = force_min_intersect_duration + s1_last_time_updated;
    }
    else {
      //std::cerr << "butt: " << first_duration_of_collision <<"," <<s1_last_time_updated << "\n";
      result = first_duration_of_collision + s1_last_time_updated;
    }
    result = round_time_downwards(result, std::max(max_meaningful_time_precision(s1_velocity), max_meaningful_time_precision(s2_velocity)));
    if (result < s1_last_time_updated) return s1_last_time_updated;
    else {
      
            shape s1tr(*s1); s1tr.translate(movement_delta_from_start_to(s1_velocity, result - s1_last_time_updated));
            shape s2tr(*s2); s2tr.translate(movement_delta_from_start_to(s2_velocity, result - s2_last_time_updated));
            assert(!s1tr.intersects(s2tr));
      return result;
    }
  /*
  //assert(!s1->intersects(*s2));
  //assert(false);
    if (auto result = get_first_moment_of_intersection_(s1, s2, s1_velocity, s2_velocity, s1_last_time_updated, s2_last_time_updated,
    vector3<fine_scalar>(
      sign(s1_velocity(X) - s2_velocity(X)) * max_error_dist,
      sign(s1_velocity(Y) - s2_velocity(Y)) * max_error_dist,
      sign(s1_velocity(Z) - s2_velocity(Z)) * max_error_dist
    ),
    0, 1)) {
      return result;
    }
    else {
      std::cerr << "Warning: Detecting no collisions when it seems like there should be one!";
      //assert(false);
    }*/
  }
  return boost::none;
}

//haaaaaaaaaaaack, copied from polygon_collision_detection.cpp!
vector3<polygon_int_type> plane_normal(vector3<polygon_int_type> p1, vector3<polygon_int_type> p2, vector3<polygon_int_type> p3) {
  return vector3<polygon_int_type>(
    (p1(Y) * (p2(Z) - p3(Z))) + (p2(Y) * (p3(Z) - p1(Z))) + (p3(Y) * (p1(Z) - p2(Z))),
    (p1(Z) * (p2(X) - p3(X))) + (p2(Z) * (p3(X) - p1(X))) + (p3(Z) * (p1(X) - p2(X))),
    (p1(X) * (p2(Y) - p3(Y))) + (p2(X) * (p3(Y) - p1(Y))) + (p3(X) * (p1(Y) - p2(Y)))
  );
}
void populate_with_plane_info(convex_polyhedron const& p, std::vector<plane_as_base_point_and_normal>& collector) {
  for (uint8_t i = 0; i < p.face_info().size(); i += p.face_info()[i] + 1) {
    collector.push_back(plane_as_base_point_and_normal(
        p.vertices()[p.face_info()[i + 1]],
        // relying on the fact that the first three vertices of each face are in the proper order.
        plane_normal(p.vertices()[p.face_info()[i + 1]],
                     p.vertices()[p.face_info()[i + 2]],
                     p.vertices()[p.face_info()[i + 3]])
      ));
  }
}

// should probably be in polygon_collision_detection.cpp instead?
inline optional_time get_first_moment_of_intersection_of_polys(convex_polyhedron const& p1, convex_polyhedron const& p2, vector3<fine_scalar> s1_velocity, vector3<fine_scalar> s2_velocity, time_type s1_last_time_updated, time_type s2_last_time_updated) {
  // Standardize: s1 is the one whose position is up-to-date.
  if (s2_last_time_updated > s1_last_time_updated) {
    return get_first_moment_of_intersection_of_polys(p2, p1, s2_velocity, s1_velocity, s2_last_time_updated, s1_last_time_updated);
  }
  
  time_type min_intersecting(s1_last_time_updated);
  time_type max_intersecting(1);
  
  const vector3<fine_scalar> relative_velocity = s1_velocity - s2_velocity;
  std::cerr << relative_velocity << "\n";
  const time_int_type prec = std::max(max_meaningful_time_precision(s1_velocity), max_meaningful_time_precision(s2_velocity));
  
  // TODO clean up duplicate code
  std::vector<plane_as_base_point_and_normal> p1_planes; p1_planes.reserve(p1.num_faces());
  populate_with_plane_info(p1, p1_planes);
  
  for (auto const& plane : p1_planes) {
    //std::cerr << plane.base_point << plane.normal << relative_velocity << "\n";
    const auto vel_dotprod = relative_velocity.dot<fine_scalar>(plane.normal);
    const polygon_rational_type before_frame_offset = polygon_rational_type(s2_velocity.dot<fine_scalar>(plane.normal), velocity_scale_factor) * (s1_last_time_updated - s2_last_time_updated);
    
    bool found_any_verts = false;
    bool excludes_entire = true;
    polygon_rational_type closest_vertex_cross;
    for (auto const& v : p2.vertices()) {
      vector3<fine_scalar> worst_error(
        -sign(plane.normal(X)) * sign(vel_dotprod),
        -sign(plane.normal(Y)) * sign(vel_dotprod),
        -sign(plane.normal(Z)) * sign(vel_dotprod));
      const auto dotprod = before_frame_offset + polygon_rational_type(((v - plane.base_point) + worst_error).dot<fine_scalar>(plane.normal));
      //std::cerr << "foo" << dotprod << "\n";
      if (vel_dotprod == 0) {
        // Either this plane completely excludes the other shape
        // ("there is no intersection")
        // or always will contain it ("we learn nothing")
        if (dotprod <= polygon_rational_type(0)) {
          excludes_entire = false;
        }
      }
      else if ((!found_any_verts) || (dotprod < closest_vertex_cross)) {
        found_any_verts = true;
        closest_vertex_cross = dotprod;
      //std::cerr << "bar" << closest_vertex_cross << "\n";
      }
    }
      
    if (vel_dotprod == 0) {
      if (excludes_entire) {
        //std::cerr << "bail\n";
        return boost::none;
      }
    }
    else {
      if (vel_dotprod < 0) {
        const time_type crossing_point = round_time_upwards(round_time_upwards(round_time_upwards(closest_vertex_cross, prec) * time_type(velocity_scale_factor, vel_dotprod), prec) + s1_last_time_updated, prec);
        if (crossing_point < max_intersecting) {
          std::cerr << min_intersecting << ", " << crossing_point << "==" << max_intersecting << "\n";
          max_intersecting = crossing_point;
          if (min_intersecting > max_intersecting) return boost::none;
        }
      }
      else {
        const time_type crossing_point = round_time_downwards(round_time_downwards(round_time_downwards(closest_vertex_cross, prec) * time_type(velocity_scale_factor, vel_dotprod), prec) + s1_last_time_updated, prec);
        if (crossing_point > min_intersecting) {
          std::cerr << min_intersecting << "==" << crossing_point << ", " << max_intersecting << "\n";
          min_intersecting = crossing_point;
          if (min_intersecting > max_intersecting) return boost::none;
        }
      }
    }
  }
  
  std::vector<plane_as_base_point_and_normal> p2_planes; p2_planes.reserve(p2.num_faces());
  populate_with_plane_info(p2, p2_planes);
  
  for (auto const& plane : p2_planes) {
    const auto vel_dotprod = relative_velocity.dot<fine_scalar>(-plane.normal);
    const polygon_rational_type before_frame_offset = polygon_rational_type(s2_velocity.dot<fine_scalar>(-plane.normal), velocity_scale_factor) * (s1_last_time_updated - s2_last_time_updated);
    
    bool found_any_verts = false;
    bool excludes_entire = true;
    polygon_rational_type closest_vertex_cross;
    for (auto const& v : p1.vertices()) {
      vector3<fine_scalar> worst_error(
        sign(plane.normal(X)) * sign(vel_dotprod),
        sign(plane.normal(Y)) * sign(vel_dotprod),
        sign(plane.normal(Z)) * sign(vel_dotprod));
      const auto dotprod = before_frame_offset + polygon_rational_type(((plane.base_point - v) + worst_error).dot<fine_scalar>(-plane.normal));
      if (vel_dotprod == 0) {
        // Either this plane completely excludes the other shape
        // ("there is no intersection")
        // or always will contain it ("we learn nothing")
        if (dotprod <= polygon_rational_type(0)) {
          excludes_entire = false;
        }
      }
      else if ((!found_any_verts) || (dotprod < closest_vertex_cross)) {
        found_any_verts = true;
        closest_vertex_cross = dotprod;
      }
    }
      
    if (vel_dotprod == 0) {
      if (excludes_entire) {
        //std::cerr << "bailii\n";
        return boost::none;
      }
    }
    else {
      if (vel_dotprod < 0) {
        const time_type crossing_point = round_time_upwards(round_time_upwards(round_time_upwards(closest_vertex_cross, prec) * time_type(velocity_scale_factor, vel_dotprod), prec) + s1_last_time_updated, prec);
        if (crossing_point < max_intersecting) {
          std::cerr << min_intersecting << ", " << crossing_point << "==" << max_intersecting << "ii\n";
          max_intersecting = crossing_point;
          if (min_intersecting > max_intersecting) return boost::none;
        }
      }
      else {
        const time_type crossing_point = round_time_downwards(round_time_downwards(round_time_downwards(closest_vertex_cross, prec) * time_type(velocity_scale_factor, vel_dotprod), prec) + s1_last_time_updated, prec);
        if (crossing_point > min_intersecting) {
          std::cerr << min_intersecting << "==" << crossing_point << ", " << max_intersecting << "ii\n";
          min_intersecting = crossing_point;
          if (min_intersecting > max_intersecting) return boost::none;
        }
      }
    }
  }
  
        //std::cerr << min_intersecting << "!\n";
  shape p1tb(p1);
  shape p2tb(p2);
  p2tb.translate(movement_delta_from_start_to(s2_velocity, s1_last_time_updated - s2_last_time_updated));
  std::cerr << "checking... ";
  if(p1tb.intersects(p2tb)) {
    std::cerr << "fail!\n";
    }
    else {
    std::cerr << "ok\n";
  shape p1t(p1);
  shape p2t(p2);
  p1t.translate(movement_delta_from_start_to(s1_velocity, min_intersecting - s1_last_time_updated));
  p2t.translate(movement_delta_from_start_to(s2_velocity, min_intersecting - s2_last_time_updated));
  assert(!p1t.intersects(p2t));
    }
  return min_intersecting;
}

inline optional_time get_first_moment_of_intersection(shape const* s1, shape const* s2, vector3<fine_scalar> s1_velocity, vector3<fine_scalar> s2_velocity, time_type s1_last_time_updated, time_type s2_last_time_updated) {
  // haaaaaack
  if (s2->get_polyhedra().empty()) {
    return get_first_moment_of_intersection_of_polys(s1->get_polyhedra().front(), convex_polyhedron(s2->get_boxes().front()), s1_velocity, s2_velocity, s1_last_time_updated, s2_last_time_updated);
  }
  else {
    return get_first_moment_of_intersection_of_polys(s1->get_polyhedra().front(), s2->get_polyhedra().front(), s1_velocity, s2_velocity, s1_last_time_updated, s2_last_time_updated);
  }
}



struct collision_info {
  collision_info(time_type t,time_type ct,object_or_tile_identifier o1,int v1,object_or_tile_identifier o2,int v2):
    time(t),computation_time(ct),oid1(o1),validation1(v1),oid2(o2),validation2(v2){}
  time_type time;
  time_type computation_time;
  object_or_tile_identifier oid1;
  int validation1;
  object_or_tile_identifier oid2;
  int validation2;
  // Backwards sense because std::priority_queue is kinda backwards.
  bool operator<(collision_info const& other)const { return other.time < time; }
};

struct moving_object_info {
  moving_object_info():invalidation_counter(0),last_time_updated(0){}
  int invalidation_counter;
  time_type last_time_updated;
};

// Move the object physically, and update its caches.
// NOTE: This does NOT change its entry in the world's collision detector!
void update_object_to_time(boost::shared_ptr<mobile_object>& obj, shape& personal_space_shape, shape& detail_shape, moving_object_info& inf, time_type time) {
  caller_error_if(time < inf.last_time_updated, "You can't go back in time!");
  if (time > inf.last_time_updated) {
    vector3<fine_scalar> delta = movement_delta_from_start_to(obj->velocity(), time - inf.last_time_updated);
    personal_space_shape.translate(delta);
    detail_shape.translate(delta);
    //std::cerr << "(this message may be out-of-date/misleading) Moving object by " << delta << "; ignoring updated already " << movement_delta_from_start_to(obj->velocity(), time) << "; full delta " << movement_delta_from_start_to(obj->velocity(), time_type(1)) << "\n";
    ++inf.invalidation_counter;
    inf.last_time_updated = time;
  }
}
void update_object_for_whole_frame(boost::shared_ptr<mobile_object>& obj, shape& personal_space_shape, shape& detail_shape) {
  vector3<fine_scalar> delta = movement_delta_from_start_to(obj->velocity(), time_type(1));
  personal_space_shape.translate(delta);
  detail_shape.translate(delta);
    //std::cerr << "!Moving object by " << delta << "\n";
}

void collect_collisions(time_type min_time, bool erase_old_sweep, object_identifier id, boost::shared_ptr<mobile_object> const& obj, shape const* personal_space_shape, world &w, objects_collision_detector& sweep_box_cd, std::priority_queue<collision_info, std::vector<collision_info>>& anticipated_collisions, std::unordered_map<object_or_tile_identifier, moving_object_info>& objects_info, objects_map<mobile_object>::type const& moving_objects) {
  if (erase_old_sweep) sweep_box_cd.erase(id);
  
  bounding_box sweep_bounds = personal_space_shape->bounds();
  auto delta = movement_delta_from_start_to(obj->velocity(), time_type(1) - min_time);
  if (delta != vector3<fine_scalar>(0,0,0)) {
    // Round up, so that our sweep bounds contains at least the area the rounded polygon sweep will contain
    sweep_bounds.translate(delta + vector3<fine_scalar>(sign(delta.x) * max_error_dist,sign(delta.y) * max_error_dist,sign(delta.z) * max_error_dist));
    sweep_bounds.combine_with(personal_space_shape->bounds());
    assert(sweep_bounds != personal_space_shape->bounds());
  }
  
  w.ensure_realization_of_space(sweep_bounds, CONTENTS_AND_LOCAL_CACHES_ONLY);

  // Did this object (1) potentially pass through another object's sweep
  // and/or (2) potentially hit a stationary object/tile?
  //
  // These checks are conservative and bounding-box based.
  vector<object_identifier> objects_this_could_collide_with;
  sweep_box_cd.get_objects_overlapping(objects_this_could_collide_with, sweep_bounds);
  w.objects_exposed_to_collision().get_objects_overlapping(objects_this_could_collide_with, sweep_bounds);
  
  for (object_identifier other_id : objects_this_could_collide_with) {
  //for (auto const& p1 : moving_objects) {
  //  object_identifier other_id = p1.first;
    if (other_id != id) {
        //std::cerr << "Considering " << id << ", " << other_id << "\n";
      shape const* opss = find_as_pointer(w.get_object_personal_space_shapes(), other_id);
      assert(opss);
      vector3<fine_scalar> other_velocity(0,0,0);
      if (boost::shared_ptr<mobile_object> const* other_obj = find_as_pointer(moving_objects, other_id)) {
        other_velocity = (*other_obj)->velocity();
        //std::cerr << "Other velocity is relevant." << obj->velocity() << (*other_obj)->velocity();
      }
      const auto i1 = objects_info.find(id);
      const time_type ltu1 = (i1 == objects_info.end()) ? time_type(0) : i1->second.last_time_updated;
      assert(ltu1 == min_time); // TODO just use this instead and not pass the min_time argument?
      const auto i2 = objects_info.find(other_id);
      const time_type ltu2 = (i2 == objects_info.end()) ? time_type(0) : i2->second.last_time_updated;
      if (optional_time result = get_first_moment_of_intersection(personal_space_shape, opss, obj->velocity(), other_velocity, ltu1, ltu2)) {
        anticipated_collisions.push(collision_info(
          *result, min_time,
          // operator[] creates the entry if needed
          object_or_tile_identifier(id), objects_info[id].invalidation_counter,
          object_or_tile_identifier(other_id), objects_info[other_id].invalidation_counter));
        //std::cerr << "Added " << id << ", " << other_id << "\n";
      }
    }
  }
  
  vector<tile_location> tiles_this_could_collide_with;
  w.get_tiles_exposed_to_collision_within(tiles_this_could_collide_with,
      get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(sweep_bounds));
  for (auto const& loc : tiles_this_could_collide_with) {
      //std::cerr << "Considering " << id << ", " << loc << "\n";
    shape t = tile_shape(loc.coords());
    const auto i1 = objects_info.find(id);
    const time_type ltu1 = (i1 == objects_info.end()) ? time_type(0) : i1->second.last_time_updated;
    if (optional_time result = get_first_moment_of_intersection(personal_space_shape, &t, obj->velocity(), vector3<fine_scalar>(0,0,0), ltu1, time_type(0))) {
      anticipated_collisions.push(collision_info(
        *result, min_time,
        // operator[] creates the entry if needed
        object_or_tile_identifier(id), objects_info[id].invalidation_counter,
        object_or_tile_identifier(loc), objects_info[loc].invalidation_counter));
      //std::cerr << "Added " << id << ", " << loc << "\n";
    }
  }

  sweep_box_cd.insert(id, sweep_bounds);
}

void assert_about_overlaps(objects_map<mobile_object>::type & moving_objects,
   object_shapes_t                  & personal_space_shapes,
   std::unordered_map<object_or_tile_identifier, moving_object_info>& objects_info, bool consider_updated_time = true) {
  
  for (auto const& p1 : moving_objects) {
    for (auto const& p2 : moving_objects) {
      if (p1.first != p2.first) {
        shape s1(personal_space_shapes.find(p1.first)->second);
        shape s2(personal_space_shapes.find(p2.first)->second);
        if (consider_updated_time) {
      const auto i1 = objects_info.find(p1.first);
      const time_type ltu1 = (i1 == objects_info.end()) ? time_type(0) : i1->second.last_time_updated;
      const auto i2 = objects_info.find(p2.first);
      const time_type ltu2 = (i2 == objects_info.end()) ? time_type(0) : i2->second.last_time_updated;
      if (ltu1 > ltu2) {
        s2.translate(movement_delta_from_start_to(p2.second->velocity(), ltu1 - ltu2));
      }
      if (ltu2 > ltu1) {
        s1.translate(movement_delta_from_start_to(p1.second->velocity(), ltu2 - ltu1));
      }
        
        //std::cerr << p1.second->velocity() << ":" << ltu1 << ", " << p2.second->velocity() << ":" << ltu2 << "\n";
      }
      else {
      //std::cerr << p1.second->velocity() << ", " << p2.second->velocity() << "\n";
      }
        assert(!s1.intersects(s2));
      }
    }
  }
}

size_t pick_random_preferring_axis_aligned(std::vector<plane_as_base_point_and_normal> const& planes, large_fast_noncrypto_rng& rng) {
  int num_axis_aligned_found = 0;
  for (auto const& p : planes) {
    if ((p.normal(X) != 0) + (p.normal(Y) != 0) + (p.normal(Z) != 0) < 2) {
      ++num_axis_aligned_found;
    }
  }
  if (num_axis_aligned_found == 0) {
    const boost::random::uniform_int_distribution<size_t> random_index(0, planes.size()-1);
    return random_index(rng);
  }
  else {
    const boost::random::uniform_int_distribution<size_t> random_index(0, num_axis_aligned_found-1);
    int which = random_index(rng);
    for (size_t i = 0; i < planes.size(); ++i) {
      auto const& p = planes[i];
      if ((p.normal(X) != 0) + (p.normal(Y) != 0) + (p.normal(Z) != 0) < 2) {
        if (which == 0) return i;
        --which;
      }
    }
  }
  assert(false);
}

void update_moving_objects_impl(
   world                            & w,
   objects_map<mobile_object>::type & moving_objects,
   object_shapes_t                  & personal_space_shapes,
   object_shapes_t                  & detail_shapes,
   objects_collision_detector       & objects_exposed_to_collision) {
   
   
  // Accelerate everything due to gravity.
  // TODO think about: Objects that are standing on the ground shouldn't
  // need to go through the whole computation rigamarole just to learn that
  // they don't end up falling through the ground.
  // Possibly omit them here somehow.
  for (auto& p : moving_objects) {
    p.second->velocity_ += gravity_acceleration;
  }
  
  objects_collision_detector sweep_box_cd;
  std::priority_queue<collision_info, std::vector<collision_info>> anticipated_collisions;
  std::unordered_map<object_or_tile_identifier, moving_object_info> objects_info;
  
  // Initialize all the objects' presumed sweeps through space this frame.
  for (auto const& p : moving_objects) {
    object_identifier const& id = p.first;
    shared_ptr<mobile_object> const& obj = p.second;
    if (shape const* personal_space_shape = find_as_pointer(personal_space_shapes, id)) {
    
      // But first, cap objects' speed in water.
      // Check any tile it happens to be in for whether there's water there.
      // If some tiles are water and some aren't, this is arbitrary, but in that case, it'll crash into a surface water on its first step, which will make this same adjustment.
      if (is_water(w.make_tile_location(get_arbitrary_containing_tile_coordinates(personal_space_shape->arbitrary_interior_point()), CONTENTS_ONLY).stuff_at().contents())) {
        const fine_scalar current_speed = obj->velocity().magnitude_within_32_bits();
        if (current_speed > max_object_speed_through_water) {
          obj->velocity_ = obj->velocity() * max_object_speed_through_water / current_speed;
        }
      }
      
      collect_collisions(time_type(0), false, id, obj, personal_space_shape, w, sweep_box_cd, anticipated_collisions, objects_info, moving_objects);
      
    }
  }
  
  
      //std::cerr << "poopPOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOPpoop!\n";
  while(!anticipated_collisions.empty()) {
    //std::cerr << "..." << anticipated_collisions.size() << "?\n";
    randomized_vector<collision_info> now_collisions;
    do {
      now_collisions.insert(anticipated_collisions.top(), w.get_rng());
      anticipated_collisions.pop();
    }
    while ((!anticipated_collisions.empty()) && (anticipated_collisions.top().time == now_collisions.begin()->time));
    
    assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
    
    //std::cerr << "..." << now_collisions.size() << "!\n";
    for (collision_info collision : now_collisions) {
      //std::cerr << "Wham!\n";
      const auto inf1_iter = objects_info.find(collision.oid1);
      assert(inf1_iter != objects_info.end());
      moving_object_info& inf1 = inf1_iter->second;
      if (collision.validation1 == inf1.invalidation_counter){
        const auto inf2_iter = objects_info.find(collision.oid2);
        assert(inf2_iter != objects_info.end());
        moving_object_info& inf2 = inf2_iter->second;
        if (collision.validation2 == inf2.invalidation_counter){
          //std::cerr << "BLAM!\n";
          // oid1 is always an object identifier
          object_identifier const* oid1p = collision.oid1.get_object_identifier(); assert(oid1p);
          object_identifier const& oid1 = *oid1p;
          boost::shared_ptr<mobile_object>* obj1p = find_as_pointer(moving_objects, oid1); assert(obj1p);
          boost::shared_ptr<mobile_object>& obj1 = *obj1p;
          shape* o1pss = find_as_pointer(personal_space_shapes, oid1); assert(o1pss);
          shape* o1ds = find_as_pointer(detail_shapes, oid1); assert(o1ds);
        
          if (tile_location const* locp = collision.oid2.get_tile_location()) {
            if (is_water(locp->stuff_at().contents())) {
              const fine_scalar current_speed = obj1->velocity().magnitude_within_32_bits();
              if (current_speed > max_object_speed_through_water) {
                update_object_to_time(obj1, *o1pss, *o1ds, inf1, collision.time);
                ++inf1.invalidation_counter; // required in case update_object_to_time did nothing
                obj1->velocity_ = obj1->velocity() * max_object_speed_through_water / current_speed;
                
                assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
              }
            }
            else {
              // We hit a solyd objyct, o noez.
              shape t = tile_shape(locp->coords());
              if (o1pss->intersects(t)) {
                // If we were already inside the object, ignore the collision;
                // we wouldn't want an infinite loop.
                // However, this should never happen.
                std::cerr << "Warning: Objects intersecting each other is deprecated.\n";
              }
              else {
                //std::cerr << inf1.last_time_updated;
                //std::cerr << collision.time;
                update_object_to_time(obj1, *o1pss, *o1ds, inf1, collision.time);
                //std::cerr << "!!!" << inf1.last_time_updated << "\n";
                // TODO FIX HACK: Assuming that mobile objects are a polyhedron and tiles a bbox
                const auto old_vel = obj1->velocity();
                std::vector<plane_as_base_point_and_normal> excluding_planes;
                find_excluding_planes(*(o1pss->get_polyhedra().begin()), convex_polyhedron(*(t.get_boxes().begin())), &excluding_planes);
                assert(!excluding_planes.empty());
                size_t which_plane = pick_random_preferring_axis_aligned(excluding_planes, w.get_rng());
                auto normal = excluding_planes[which_plane].normal;
                auto vel_change = (normal * obj1->velocity().dot<fine_scalar>(normal));
                if (vel_change != vector3<fine_scalar>(0,0,0)) {
                  fine_scalar vel_change_denom = normal.dot<fine_scalar>(normal);
                  // Hack: To avoid getting locked in a nonzero velocity due to rounding error,
                  // make sure to round down your eventual velocity!
                  //std::cerr << obj1->velocity();
                  obj1->velocity_ = ((obj1->velocity() * vel_change_denom) - vel_change) / vel_change_denom;
                  //std::cerr << vel_change << vel_change_denom << obj1->velocity() << "\n";
                }
                // Also push the objects away from each other a little if the surface isn't axis-aligned.
                //if (((normal(X) != 0) + (normal(Y) != 0) + (normal(Z) != 0)) > 1) {
                //  
                //}
                if (obj1->velocity() != old_vel) ++inf1.invalidation_counter;
                else {
                  std::cerr << "Warning: No velocity change on collision. This can potentially cause overlaps.\n";
                }
                
                assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
              }
            }
          }
          if (object_identifier const* oid2p = collision.oid2.get_object_identifier()) {
            object_identifier const& oid2 = *oid2p;
            boost::shared_ptr<mobile_object>* obj2p = find_as_pointer(moving_objects, oid2); assert(obj2p);
            boost::shared_ptr<mobile_object>& obj2 = *obj2p;
            shape* o2pss = find_as_pointer(personal_space_shapes, oid2); assert(o2pss);
            shape* o2ds = find_as_pointer(detail_shapes, oid2); assert(o2ds);
            
            // CRASH!!
            //std::cerr << oid1 << oid2;
            // This assertion is wrong:
            //assert (!o1pss->intersects(*o2pss));
            // Because one object can follow another closely, with the one in front known to be
            // not colliding with anything else until after the time the back one intersects
            // where the front one was.
            /*these are no longer needed OR WANTED
            if (inf1.last_time_updated < collision.computation_time)
              update_object_to_time(obj1, *o1pss, *o1ds, inf1, collision.computation_time);
            if (inf2.last_time_updated < collision.computation_time)
              update_object_to_time(obj2, *o2pss, *o2ds, inf2, collision.computation_time);
            assert (!o1pss->intersects(*o2pss) && true);*/
            update_object_to_time(obj1, *o1pss, *o1ds, inf1, collision.time);
            ++inf1.invalidation_counter; // required in case update_object_to_time did nothing
            update_object_to_time(obj2, *o2pss, *o2ds, inf2, collision.time);
            ++inf2.invalidation_counter; // required in case update_object_to_time did nothing
            assert (!o1pss->intersects(*o2pss));
            assert_about_overlaps(moving_objects, personal_space_shapes,objects_info);
            
            // TODO FIX HACK: Assuming that mobile objects are one polyhedron
            std::vector<plane_as_base_point_and_normal> excluding_planes;
            find_excluding_planes(*(o1pss->get_polyhedra().begin()), convex_polyhedron(*(o2pss->get_polyhedra().begin())), &excluding_planes);
            assert(!excluding_planes.empty());
            size_t which_plane = pick_random_preferring_axis_aligned(excluding_planes, w.get_rng());
            auto normal = excluding_planes[which_plane].normal;
            // Hack: To avoid getting locked in a nonzero velocity due to rounding error,
            // make sure to round down your eventual velocity!
            auto veldiff_num = (normal * (obj1->velocity() - obj2->velocity()).dot<fine_scalar>(normal)) * 2;
            auto veldiff_denom = normal.dot<fine_scalar>(normal) * 3;
            obj1->velocity_ = ((obj1->velocity_ * veldiff_denom) - veldiff_num) / veldiff_denom;
            obj2->velocity_ = ((obj2->velocity_ * veldiff_denom) + veldiff_num) / veldiff_denom;
            
            //obj1->velocity_ = (obj1->velocity() + obj2->velocity()) / 2;
            //obj2->velocity_ = obj1->velocity();
          
            // If we updated anything about the objects, they might have new collisions.
            // If we *didn't*, they won't repeat the same collision again, since we have
            //     deleted it and not replaced it.
            // Note: Duplicate comment with the below
            if (inf2.invalidation_counter > collision.validation2) {
              collect_collisions(collision.time, true, oid2, obj2, o2pss, w, sweep_box_cd, anticipated_collisions, objects_info, moving_objects);
            }
          }
          // If we updated anything about the objects, they might have new collisions.
          // If we *didn't*, they won't repeat the same collision again, since we have
          //     deleted it and not replaced it.
          // Note: Duplicate comment with the above
          if (inf1.invalidation_counter > collision.validation1) {
            collect_collisions(collision.time, true, oid1, obj1, o1pss, w, sweep_box_cd, anticipated_collisions, objects_info, moving_objects);
          }
        }
      }
    }
  }
  
  assert_about_overlaps(moving_objects, personal_space_shapes, objects_info);
  //std::cerr << "Excellently.\n";
  for (auto const& p : moving_objects) {
    auto const& oid = p.first;
    boost::shared_ptr<mobile_object>* objp = find_as_pointer(moving_objects, oid); assert(objp);
    boost::shared_ptr<mobile_object>& obj = *objp;
    shape* opss = find_as_pointer(personal_space_shapes, oid); assert(opss);
    shape* ods = find_as_pointer(detail_shapes, oid); assert(ods);
    auto inf = objects_info.find(object_or_tile_identifier(oid));
    if (inf == objects_info.end()) {
      update_object_for_whole_frame(obj, *opss, *ods);
      //std::cerr << "Minimally so.\n";
    }
    else {
      update_object_to_time(obj, *opss, *ods, inf->second, time_type(1));
      //std::cerr << "Quite so.\n";
    }
    
    // Update the collision detector entries
    bounding_box new_bounds = opss->bounds();
    new_bounds.combine_with(ods->bounds());
    objects_exposed_to_collision.erase(oid);
    objects_exposed_to_collision.insert(oid, new_bounds);
  }
  assert_about_overlaps(moving_objects, personal_space_shapes, objects_info, false);
}

} /* end anonymous namespace */

void world::update_moving_objects() {
  update_moving_objects_impl(*this, moving_objects_, object_personal_space_shapes_, object_detail_shapes_, objects_exposed_to_collision_);
}


