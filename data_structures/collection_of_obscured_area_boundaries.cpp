/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012

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

#include "collection_of_obscured_area_boundaries.hpp"

namespace collection_of_obscured_area_boundaries_impl {

void logical_node_lines_collection::note_hline_in_left_side(axis_aligned_line const& hline) {
  if ((hline.b1 <= bounds.min_x) && (hline.b2 >= bounds.min_x)) {
    left_side.tpts.insert(tpt<>(hline.l, hline.is_obscured_beyond));
  }
}
void logical_node_lines_collection::note_pline_in_left_side(perspective_line const& pline) {
  const coord_type y = pline.y_intercept(bounds.min_x);
  if ((y >= bounds.min_y) && (y <= bounds.max_y) && pline.min_x() <= bounds.min_x) {
    left_side.tpts.insert(tpt<>(y, pline.is_obscured_above()));
  }
}
// Only regenerates the tpts; you have to keep track of is_obscured_initially on your own,
// since it can't always be inferred
void logical_node_lines_collection::regenerate_left_side_tpts() {
  left_side.tpts.clear();
  for (auto const& l : hlines) note_hline_in_left_side(l);
  for (auto const& l : plines) note_pline_in_left_side(l);
}

void logical_node_lines_collection::insert_hline(axis_aligned_line const& hline) {
  assert(hline.b1 <= bounds.max_x);
  assert(hline.b2 >= bounds.min_x);
  assert(hline.l <= bounds.max_y);
  assert(hline.l >= bounds.min_y);
  hlines.push_back(hline);
  note_hline_in_left_side(hline);
}
void logical_node_lines_collection::insert_vline(axis_aligned_line const& vline) {
  assert(vline.b1 <= bounds.max_y);
  assert(vline.b2 >= bounds.min_y);
  assert(vline.l <= bounds.max_x);
  assert(vline.l >= bounds.min_x);
  vlines.push_back(vline);
}
void logical_node_lines_collection::insert_pline(perspective_line const& pline) {
  assert((pline.b1.is_x_boundary ? pline.b1.b : pline.x_intercept(pline.b1.b))
    < (pline.b2.is_x_boundary ? pline.b2.b : pline.x_intercept(pline.b2.b)));
  plines.push_back(pline);
  note_pline_in_left_side(pline);
}

void logical_node_lines_collection::copy_from_same_or_child(logical_node_lines_collection const& child_collection) {
  for (auto const& hline : child_collection.hlines) {
    insert_hline(hline);
  }
  for (auto const& vline : child_collection.vlines) {
    insert_vline(vline);
  }
  for (auto const& pline : child_collection.plines) {
    insert_pline(pline);
  }
  if ((child_collection.bounds.min_x == bounds.min_x) && (child_collection.bounds.min_y == bounds.min_y)) {
    left_side.is_obscured_initially = child_collection.left_side.is_obscured_initially;
  }
}

// Copy another collection, but crop to a certain set of bounds.
logical_node_lines_collection::logical_node_lines_collection(logical_node_lines_collection const& original, bounds_2d bounds):bounds(bounds) {
  /* We could copy their left-side info, but it's easy to just let it be generated by the insert functions.
    
  if (bounds.min_x == original.bounds.min_x) {
    left_side.tpts.insert(original.left_side.tpts.lower_bound(tpt<>(bounds.min_y, false)), original.left_side.tpts.upper_bound(tpt<>(max_y, false)));
    if (left_side.tpts.empty()) {
      left_side.is_obscured_initially = original.left_side.point_is_obscured(bounds.min_y);
    }
    else {
      left_side.is_obscured_initially = !left_side.begin().is_obscured_beyond;
    }
  }
  else {
  }*/
  
  left_side.is_obscured_initially = original.point_is_obscured(bounds.min_x, bounds.min_y);
  
  for (auto const& hline : original.hlines) {
    if ((hline.b1 <= bounds.max_x) && (hline.b2 >= bounds.min_x)
     && (hline.l  <= bounds.max_y) && (hline.l  >= bounds.min_y)) {
      insert_hline(hline);
    }
  }
  for (auto const& vline : original.vlines) {
    if ((vline.b1 <= bounds.max_y) && (vline.b2 >= bounds.min_y)
     && (vline.l  <= bounds.max_x) && (vline.l  >= bounds.min_x)) {
      insert_vline(vline);
    }
  }
  for (auto const& pline : original.plines) {
    if ((pline.min_x() <= bounds.max_x) && (pline.min_y() <= bounds.max_y) &&
        (pline.max_x() >= bounds.min_x) && (pline.max_y() >= bounds.min_y)) {
      const coord_type lyx = pline.x_intercept(bounds.min_y);
      const coord_type myx = pline.x_intercept(bounds.max_y);
      if (((lyx <= bounds.max_x) && (myx >= bounds.min_x))
       || ((lyx >= bounds.min_x) && (myx <= bounds.max_x))) {
        insert_pline(pline);
      }
    }
  }
}

bool logical_node_lines_collection::point_is_obscured(coord_type x, coord_type y)const {
  // Effectively: Draw a horizontal line back to the left side. If it intersects any lines,
  // then the closest intersection dictates whether you're obscured or not.
  // Otherwise, check the left_side structure to see whether you're obscured.
  assert((x >= bounds.min_x) && (y >= bounds.min_y) && (x <= bounds.max_x) && (y <= bounds.max_y));
  bool found_any_lines = false;
  coord_type best_x;
  bool best_obscuredness;
  for (auto const& vline : vlines) {
    assert(vline.l >= bounds.min_x);
    if ((vline.l <= x) && (vline.b1 <= y) && (vline.b2 >= y)) {
      if ((!found_any_lines) || (vline.l > best_x)) {
        found_any_lines = true;
        best_x = vline.l;
        best_obscuredness = vline.is_obscured_beyond;
      }
    }
  }
  for (auto const& pline : plines) {
    const coord_type pline_x = pline.x_intercept(y);
    if ((pline_x <= x) && (pline_x >= bounds.min_x) && pline.point_is_within_end_boundaries(pline_x, y)) {
      if ((!found_any_lines) || (pline_x > best_x)) {
        found_any_lines = true;
        best_x = pline_x;
        best_obscuredness = pline.is_obscured_to_the_right();
      }
    }
  }
  if (found_any_lines) {
    return best_obscuredness;
  }
  else {
    return left_side.point_is_obscured(y);
  }
}


// Utility functions for logical_node_lines_collection::relate():

bool logical_node_lines_collection::handle_aaline_vs_aaline(axis_aligned_line l1, obscured_areas_tracker_1d<>& l1_info, axis_aligned_line l2, obscured_areas_tracker_1d<>& l2_info) {
  // Note: If the rules have been followed, they can never intersect outside the box,
  // so we don't need to check that the point of intersection is inside the box.
  if (intersects(l1, l2)) {
    l1_info.insert(tpt<>(l2.l, l2.is_obscured_beyond));
    l2_info.insert(tpt<>(l1.l, l1.is_obscured_beyond));
    return true;
  }
  else {
    return false;
  }
}
  
bool logical_node_lines_collection::handle_aaline_vs_pline(bool aaline_is_horizontal, axis_aligned_line aaline, obscured_areas_tracker_1d<>& aaline_info, perspective_line pline, obscured_areas_tracker_1d<pline_tpt_loc>& pline_info) {
  coord_type b = (aaline_is_horizontal ? pline.x_intercept(aaline.l) : pline.y_intercept(aaline.l));
  // Make sure the point of intersection is inside the box...
  if (aaline_is_horizontal) {
    if ((       b < bounds.min_x) || (       b > bounds.max_x) ||
        (aaline.l < bounds.min_y) || (aaline.l > bounds.max_y)) return false;
  }
  else {
    if ((       b < bounds.min_y) || (       b > bounds.max_y) ||
        (aaline.l < bounds.min_x) || (aaline.l > bounds.max_x)) return false;
  }
  if ((aaline.b1 < b) && (b < aaline.b2) && pline.point_is_within_end_boundaries((aaline_is_horizontal ? b : aaline.l), (aaline_is_horizontal ? aaline.l : b))) {
    aaline_info.insert(tpt<>(b, (
        aaline_is_horizontal ? pline.is_obscured_to_the_right() : pline.is_obscured_above())));
    pline_info.insert(tpt<pline_tpt_loc>(
        pline_tpt_loc(aaline.l, !aaline_is_horizontal, &pline),
        aaline.is_obscured_beyond == (pline.positive_slope || !aaline_is_horizontal)
      ));
    return true;
  }
  else {
    return false;
  }
}
  
void logical_node_lines_collection::mark_obscured_aalines(logical_node_lines_collection const& other, bool aalines_are_horizontal, bool src_aalines_is_our_own_collection, std::vector<axis_aligned_line> const& src_aalines, std::vector<obscured_areas_tracker_1d<>> const& tpts, std::vector<size_t>& completely_obscured_or_completely_visible_list) {
  assert(bounds == other.bounds);
  for (size_t i = 0; i < tpts.size(); ++i) {
    if (tpts[i].tpts.empty()) {
      axis_aligned_line const& aaline = src_aalines[i];
      bool is_obscured;
      if (aalines_are_horizontal) {
        is_obscured = other.point_is_obscured((aaline.b1 > bounds.min_x) ? aaline.b1 : bounds.min_x, aaline.l);
      }
      else {
        is_obscured = other.point_is_obscured(aaline.l, (aaline.b1 > bounds.min_y) ? aaline.b1 : bounds.min_y);
      }
      if (is_obscured == src_aalines_is_our_own_collection) {
        completely_obscured_or_completely_visible_list.push_back(i);
      }
    }
  }
}
void logical_node_lines_collection::mark_obscured_plines(logical_node_lines_collection const& other, bool src_plines_is_our_own_collection, std::vector<perspective_line> const& src_plines, std::vector<obscured_areas_tracker_1d<pline_tpt_loc>> const& tpts, std::vector<size_t>& completely_obscured_or_completely_visible_list) {
  assert(bounds == other.bounds);
  for (size_t i = 0; i < tpts.size(); ++i) {
    if (tpts[i].tpts.empty()) {
      perspective_line const& pline = src_plines[i];
      coord_type pline_check_x = pline.min_x();
      coord_type pline_check_y = pline.start_y();
      if (pline_check_x < bounds.min_x) {
        pline_check_x = bounds.min_x;
        pline_check_y = pline.y_intercept(pline_check_x);
      }
      if (pline.positive_slope && pline_check_y < bounds.min_y) {
        pline_check_y = bounds.min_y;
        pline_check_x = pline.x_intercept(pline_check_y);
      }
      if ((!pline.positive_slope) && pline_check_y > bounds.max_y) {
        pline_check_y = bounds.max_y;
        pline_check_x = pline.x_intercept(pline_check_y);
      }
      if (other.point_is_obscured(pline_check_x, pline_check_y) == src_plines_is_our_own_collection) {
        completely_obscured_or_completely_visible_list.push_back(i);
      }
    }
  }
}

template<class LineType, typename TptLocType> void slice_marked_lines(bool src_lines_is_our_own_collection, std::vector<LineType> const& src_lines, std::vector<LineType>& dst_lines, std::vector<obscured_areas_tracker_1d<TptLocType>> const& tpts) {
  for (size_t i = 0; i < tpts.size(); ++i) {
    auto const& switch_points = tpts[i].tpts;
    auto t = switch_points.begin();
    if (t != switch_points.end()) {
      LineType const& line = src_lines[i];
      // If we're in the dst collection, replace ourselves with the first still-visible piece of ourselves.
      auto original_b2 = line.b2;
      if (t->is_obscured_beyond) {
        if (src_lines_is_our_own_collection) dst_lines[i].b2 = t->loc;
        else dst_lines.push_back(LineType(line.projective_angle(), line.b1, t->loc, line.is_obscured_beyond));
      }
      else {
        auto new_b1 = t->loc;
        if (src_lines_is_our_own_collection) dst_lines[i].b1 = new_b1;
        ++t;
        if (t != switch_points.end()) {
          //TODO: This assertion might fail if rounding error stuff happens. What to do about that?
          assert(t->is_obscured_beyond);
          
          if (src_lines_is_our_own_collection) dst_lines[i].b2 = t->loc;
          else dst_lines.push_back(LineType(line.projective_angle(), new_b1, t->loc, line.is_obscured_beyond));
        }
        else {
          if(!src_lines_is_our_own_collection) dst_lines.push_back(LineType(line.projective_angle(), new_b1, original_b2, line.is_obscured_beyond));
        }
      }
      
      // Now insert the rest of the still-visible pieces of ourselves at the end.
      while(t != switch_points.end()) {
        ++t;
        if (t == switch_points.end()) break;
        
        //TODO: This assertion might fail if rounding error stuff happens. What to do about that?
        assert(!t->is_obscured_beyond);
        auto startb = t->loc;
        ++t;
        if (t == switch_points.end()) {
          dst_lines.push_back(LineType(line.projective_angle(), startb, original_b2, line.is_obscured_beyond));
        }
        else {
          //TODO: This assertion might fail if rounding error stuff happens. What to do about that?
          assert(t->is_obscured_beyond);
          dst_lines.push_back(LineType(line.projective_angle(), startb, t->loc, line.is_obscured_beyond));
        }
      }
    }
  }
}
template<class LineType> void purge_obscured_lines(std::vector<LineType>& lines, std::vector<size_t> const& obscured_list) {
  for (int i = obscured_list.size() - 1; i >= 0; --i) {
    lines[obscured_list[i]] = lines.back(); lines.pop_back();
  }
}
template<class LineType> void copy_visible_lines(std::vector<LineType> const& src_lines, std::vector<LineType>& dst_lines, std::vector<size_t> const& visible_list) {
  //for (auto l : src_lines) dst_lines.push_back(l);
  for (size_t i : visible_list) {
    dst_lines.push_back(src_lines[i]);
  }
}

bool logical_node_lines_collection::relate(logical_node_lines_collection const& other, bool insert) {
  assert(bounds == other.bounds);
  bool result = false;
  
  // ======== Combine the lines info. ==========
  // First, collect all overlaps.
  std::vector<obscured_areas_tracker_1d<>>               hlines_tpts(      hlines.size());
  std::vector<obscured_areas_tracker_1d<>>              ohlines_tpts(other.hlines.size());
  std::vector<obscured_areas_tracker_1d<>>               vlines_tpts(      vlines.size());
  std::vector<obscured_areas_tracker_1d<>>              ovlines_tpts(other.vlines.size());
  std::vector<obscured_areas_tracker_1d<pline_tpt_loc>>  plines_tpts(      plines.size());
  std::vector<obscured_areas_tracker_1d<pline_tpt_loc>> oplines_tpts(other.plines.size());
  std::vector<size_t> completely_obscured_hlines;
  std::vector<size_t> completely_obscured_vlines;
  std::vector<size_t> completely_obscured_plines;
  std::vector<size_t> completely_visible_ohlines;
  std::vector<size_t> completely_visible_ovlines;
  std::vector<size_t> completely_visible_oplines;
  for (size_t i = 0; i < hlines.size(); ++i) {
    for (size_t j = 0; j < other.vlines.size(); ++j) {
      if (handle_aaline_vs_aaline(hlines[i], hlines_tpts[i], other.vlines[j], ovlines_tpts[j])) {
        if (insert) { result = true; } else { return true; }
      }
    }
    for (size_t j = 0; j < other.plines.size(); ++j) {
      if (handle_aaline_vs_pline(true, hlines[i], hlines_tpts[i], other.plines[j], oplines_tpts[j])) {
        if (insert) { result = true; } else { return true; }
      }
    }
  }
  for (size_t i = 0; i < vlines.size(); ++i) {
    for (size_t j = 0; j < other.hlines.size(); ++j) {
      if (handle_aaline_vs_aaline(vlines[i], vlines_tpts[i], other.hlines[j], ohlines_tpts[j])) {
        if (insert) { result = true; } else { return true; }
      }
    }
    for (size_t j = 0; j < other.plines.size(); ++j) {
      if (handle_aaline_vs_pline(false, vlines[i], vlines_tpts[i], other.plines[j], oplines_tpts[j])) {
        if (insert) { result = true; } else { return true; }
      }
    }
  }
  for (size_t i = 0; i < plines.size(); ++i) {
    for (size_t j = 0; j < other.hlines.size(); ++j) {
      if (handle_aaline_vs_pline(true, other.hlines[j], ohlines_tpts[j], plines[i], plines_tpts[i])) {
        if (insert) { result = true; } else { return true; }
      }
    }
    for (size_t j = 0; j < other.vlines.size(); ++j) {
      if (handle_aaline_vs_pline(false, other.vlines[j], ovlines_tpts[j], plines[i], plines_tpts[i])) {
        if (insert) { result = true; } else { return true; }
      }
    }
  }
    
  // For lines that don't have overlaps, note whether or not they're completely obscured.
  // They can only be completely obscured *by the other collection*
  //  (since each line is completely necessary to its own collection)
  // and since we haven't modified either collection yet, we can just ask the other collection
  // whether a point on that line is visible.
  mark_obscured_aalines(other, true , true ,       hlines,  hlines_tpts, completely_obscured_hlines);
  mark_obscured_aalines(other, false, true ,       vlines,  vlines_tpts, completely_obscured_vlines);
  mark_obscured_plines (other,        true ,       plines,  plines_tpts, completely_obscured_plines);
  mark_obscured_aalines(*this, true , false, other.hlines, ohlines_tpts, completely_visible_ohlines);
  mark_obscured_aalines(*this, false, false, other.vlines, ovlines_tpts, completely_visible_ovlines);
  mark_obscured_plines (*this,        false, other.plines, oplines_tpts, completely_visible_oplines);
  if (!(
      completely_obscured_hlines.empty() && completely_obscured_vlines.empty() &&
      completely_obscured_plines.empty() && completely_visible_ohlines.empty() &&
      completely_visible_ovlines.empty() && completely_visible_oplines.empty())) {
    if (insert) { result = true; } else { return true; }
  }
  
  // If nothing from the other set was visible at all, we can skip the rest of this.
  if (!result) return false;
  
  assert(insert);
  
  // Then slice up the overlapped lines based on the overlaps.
  slice_marked_lines(true ,       hlines, hlines,  hlines_tpts);
  slice_marked_lines(true ,       vlines, vlines,  vlines_tpts);
  slice_marked_lines(true ,       plines, plines,  plines_tpts);
  slice_marked_lines(false, other.hlines, hlines, ohlines_tpts);
  slice_marked_lines(false, other.vlines, vlines, ovlines_tpts);
  slice_marked_lines(false, other.plines, plines, oplines_tpts);
  
  // When you're done, purge the lines that were completely obscured.
  // And copy over the completely-visible lines from the other list.
  purge_obscured_lines(hlines, completely_obscured_hlines);
  purge_obscured_lines(vlines, completely_obscured_vlines);
  purge_obscured_lines(plines, completely_obscured_plines);
  copy_visible_lines(other.hlines, hlines, completely_visible_ohlines);
  copy_visible_lines(other.vlines, vlines, completely_visible_ovlines);
  copy_visible_lines(other.plines, plines, completely_visible_oplines);
  
  // Regenerate the left-side info.
  // (We can't meaningfully combine it because sides can have been deleted.)
  left_side.is_obscured_initially = left_side.is_obscured_initially || other.left_side.is_obscured_initially;
  regenerate_left_side_tpts();
  
  return true;
}





/*logical_node::logical_node(implementation_node* i,uint8_t l,uint32_t w,coord_int_type nx,coord_int_type ny,coord_int_type d)
    :
    impnode(i),
    which_entry_at_this_subimpnode_level(w),
    subimpnode_level(l),
    x_lower_bound_numerator(nx),
    y_lower_bound_numerator(ny),
    bounds_denominator(d)
{
  assert(subimpnode_level <= max_subimpnode_level);
  if (subimpnode_level == 0) {
    contents_type = impnode->top_entry;
  }
  else {
    which_byte_in_impnode = (total_entries_above_subimpnode_level(subimpnode_level) / 4) + (which_entry_at_this_subimpnode_level/4);
    which_bits_in_byte = 2*(which_entry_at_this_subimpnode_level % 4);
    assert(which_byte_in_impnode < total_entry_bytes);
    contents_type = (impnode->entry_bytes[which_byte_in_impnode] >> which_bits_in_byte) & 0x3;
  }
}*/

bounds_2d logical_node::bounds()const {
  if (bounds_denominator == 0) {
    return top_node_bounds;
  }
  else {
    return bounds_2d(
      coord_type(x_lower_bound_numerator  ,bounds_denominator),
      coord_type(y_lower_bound_numerator  ,bounds_denominator),
      coord_type(x_lower_bound_numerator+1,bounds_denominator),
      coord_type(y_lower_bound_numerator+1,bounds_denominator)
    );
  }
}

void logical_node::set_contents_type_bits(uint8_t type) {
  contents_type = type;
  if (subimpnode_level == 0) {
    impnode->top_entry = type;
  }
  else {
    assert(which_byte_in_impnode < total_entry_bytes);
    impnode->entry_bytes[which_byte_in_impnode] = (impnode->entry_bytes[which_byte_in_impnode] & (~(0x3 << which_bits_in_byte))) | (type << which_bits_in_byte);
  }
}

logical_node_lines_collection*& logical_node::lines_pointer_reference()const {
  assert(contents_type == MIXED_AS_LINES);
    
  const uint32_t which_entry_at_bottom_subimpnode_level = first_entry_at_bottom_subimpnode_level();
  assert(which_entry_at_bottom_subimpnode_level < total_entries_at_bottom_subimpnode_level - 3);
  assert((contents_type != MIXED_AS_CHILDREN) || (subimpnode_level == max_subimpnode_level) || (
    (which_entry_at_bottom_subimpnode_level == child(0).first_entry_at_bottom_subimpnode_level()) &&
    (which_entry_at_bottom_subimpnode_level != child(1).first_entry_at_bottom_subimpnode_level()) &&
    (which_entry_at_bottom_subimpnode_level != child(2).first_entry_at_bottom_subimpnode_level()) &&
    (which_entry_at_bottom_subimpnode_level != child(3).first_entry_at_bottom_subimpnode_level())
  ));
    
  return reinterpret_cast<logical_node_lines_collection*&>(impnode->lines_or_further_children[which_entry_at_bottom_subimpnode_level]);
}

logical_node logical_node::child(int which_child)const {
  assert(contents_type == MIXED_AS_CHILDREN);
  
  const uint32_t which_entry_at_next_subimpnode_level = which_entry_at_this_subimpnode_level * 4 + which_child;
  const coord_int_type new_denom = (bounds_denominator == 0) ? 1 : (bounds_denominator * 2);
  const coord_int_type new_xnum = ((bounds_denominator == 0) ? -1 : (x_lower_bound_numerator * 2)) + (which_child & 0x1);
  const coord_int_type new_ynum = ((bounds_denominator == 0) ? -1 : (y_lower_bound_numerator * 2)) + ((which_child & 0x2) >> 1);
  
  if (subimpnode_level == max_subimpnode_level) {
    assert(which_entry_at_next_subimpnode_level < total_entries_at_bottom_subimpnode_level);
    assert(impnode->lines_or_further_children[which_entry_at_next_subimpnode_level] != NULL);
    return logical_node(static_cast<implementation_node*>(impnode->lines_or_further_children[which_entry_at_next_subimpnode_level]), 0, 0, new_xnum, new_ynum, new_denom);
  }
  else {
    return logical_node(impnode, subimpnode_level + 1, which_entry_at_next_subimpnode_level, new_xnum, new_ynum, new_denom);
  }
}




// If there get to be too many lines in this node [how many?], split into children.
/*
   How many is "too many"?
   Well, when is it a problem to have too many?
   The answer is that we iterate through the lines *exactly when we insert another polygon whose lines intersect this box*.
   For boxes that are sufficiently small compared to the length of the lines being inserted, the chance of a line hitting the box (if we make certain assumptions about how the line is "randomly" picked) is proportional to the "diameter" of the box.
   So the average-case complexity of "having N lines in this box" when inserting a polygon[1] is O(N * width of the box).
   So if we require N < c/(width of the box) (for some constant c), then the average-case complexity is O(c/(width of the box) * width of the box) = O(c) = O(1).
   
   We could restrict the number further, but having too many nested layers of children has a memory cost and is unproductive. Consider the case where there are M long lines "very very close" to each other; if the "too many" value doesn't exceed M soon enough, we will break down into many, many layers of children. But since the size of the box halves with each level, our rule allows twice as many lines at each level, so we only get down to level log2(M), so we only create about M boxes, no matter how big M is.
   
   [1] but it's not "per instance of inserting a polygon", because half the time when the (assumed to be very large compared with this box) polygon doesn't have an edge that hits this box, it instead *completely contains* this box and this box's info gets deleted. So the O(1) at the end of the paragraph ends up being a constant cost /for each time a box is created/ and (in the average case) is not proportional to the number of additional polygons that will be added.
   HOWEVER, TAKE NOTE: if you do a lot of lookups that *aren't* insertions, this logic doesn't apply. There are some worst-case O(number of lines inserted)-per-lookup cases that can't be fixed by adjusting the "too much" number.
*/
const coord_int_type lines_intolerance_constant = 4;
void logical_node::split_if_necessary() {
  if ((contents_type == MIXED_AS_LINES) && (lines_intolerance_constant * static_cast<coord_int_type>(lines_pointer_reference()->size()) > bounds_denominator)) {
    set_contents_type(MIXED_AS_CHILDREN); // This automatically splits the lines we have up among our children.
    for (int which_child = 0; which_child < 4; ++which_child) {
      child(which_child).split_if_necessary();
    }
  }
}


/*bool collection_of_obscured_area_boundaries::insert_collection(collection_of_obscured_area_boundaries const& other) {
  return top_logical_node().insert_from_other_collection(other.top_logical_node())
}*/


// A utility function for set_contents_type.
void logical_node::retrieve_all_lines(logical_node_lines_collection& collector) {
  if (contents_type == MIXED_AS_LINES) {
    collector.copy_from_same_or_child(*lines_pointer_reference());
  }
  if (contents_type == MIXED_AS_CHILDREN) {
    for (int which_child = 0; which_child < 4; ++which_child) {
      child(which_child).retrieve_all_lines(collector);
    }
  }
}
// A utility function for set_contents_type.
void logical_node::clear_children() {
  // Clear the children, recursively. (The point of doing this is free anything behind
  //                        the bottom-level pointers.)
  for (int which_child = 0; which_child < 4; ++which_child) {
    // We need to overwrite them to ALL_CLEAR first even if we're going to delete the lower impnodes,
    // because the lower impnodes might have lower impnodes of their own.
    child(which_child).set_contents_type(ALL_CLEAR);
    
    if (subimpnode_level == max_subimpnode_level) {
      const uint32_t which_entry_at_bottom_subimpnode_level = which_entry_at_this_subimpnode_level * 4 + which_child;
      assert(which_entry_at_bottom_subimpnode_level < total_entries_at_bottom_subimpnode_level);
      implementation_node*& next_impnode = reinterpret_cast<implementation_node*&>(impnode->lines_or_further_children[which_entry_at_bottom_subimpnode_level]);
      delete next_impnode;
      next_impnode = NULL;
    }
  }
}
// A utility function for set_contents_type.
void logical_node::create_children(uint8_t children_contents_type) {
  assert(children_contents_type != MIXED_AS_CHILDREN);
  for (int which_child = 0; which_child < 4; ++which_child) {
    // We might have to create pointers for our new children before fetching them.
    if (subimpnode_level == max_subimpnode_level) {
      const uint32_t which_entry_at_bottom_subimpnode_level = which_entry_at_this_subimpnode_level * 4 + which_child;
      assert(which_entry_at_bottom_subimpnode_level < total_entries_at_bottom_subimpnode_level);
      implementation_node*& next_impnode = reinterpret_cast<implementation_node*&>(impnode->lines_or_further_children[which_entry_at_bottom_subimpnode_level]);
      assert(next_impnode == NULL);
      next_impnode = new implementation_node;
    }
    
    child(which_child).set_contents_type_bits(children_contents_type);
    if (children_contents_type == MIXED_AS_LINES) assert(child(which_child).lines_pointer_reference() == NULL);
  }
}


void logical_node::set_contents_type(uint8_t new_type, bool keep_line_info) {
  // Note: Changing all-clear or all-blocked to MIXED_AS_CHILDREN or MIXED_AS_LINES
  // creates a situation where the type is marked as MIXED but the contents aren't actually
  // mixed; you need to immediately add lines / modify a child, or it'll mess up.
  
  if (new_type == contents_type) return;
  
  // The most complicated cases: Changing MIXED_AS_CHILDREN to MIXED_AS_LINES or vice versa.
  if (keep_line_info && (contents_type == MIXED_AS_CHILDREN) && (new_type == MIXED_AS_LINES)) {
    // We have to merge the lines containers,
    // and also the children, or children's lines containers, might be behind
    // the SAME POINTER that we're going to use for our own new lines container.
    logical_node_lines_collection* combined_lines = new logical_node_lines_collection(bounds());
    retrieve_all_lines(*combined_lines);
    clear_children();
    set_contents_type_bits(MIXED_AS_LINES);
    logical_node_lines_collection*& lpr = lines_pointer_reference();
    assert(lpr == NULL);
    lpr = combined_lines;
    
    assert(false); // Remove this if we ever actually want to use this code
  }
  else if (keep_line_info && (contents_type == MIXED_AS_LINES) && (new_type == MIXED_AS_CHILDREN)) {
    logical_node_lines_collection*& lpr = lines_pointer_reference();
    logical_node_lines_collection* original_lines_ptr = lpr;
    lpr = NULL;
    set_contents_type_bits(MIXED_AS_CHILDREN);
    create_children(MIXED_AS_LINES);
    for (int which_child = 0; which_child < 4; ++which_child) {
      logical_node c = child(which_child);
      
      logical_node_lines_collection*& clpr = c.lines_pointer_reference();
      assert(clpr == NULL);
      clpr = new logical_node_lines_collection(*original_lines_ptr, c.bounds());
      
      if (clpr->empty()) c.set_contents_type(ALL_CLEAR);
      else if (clpr->full()) c.set_contents_type(ALL_BLOCKED);
    }
    delete original_lines_ptr;
  }
  else {
    // Either there's no line info to be kept, or we've been instructed not to keep it.
    if (contents_type == MIXED_AS_CHILDREN) {
      clear_children();
    }
    if (contents_type == MIXED_AS_LINES) {
      logical_node_lines_collection*& lpr = lines_pointer_reference();
      delete lpr;
      lpr = NULL;
    }
    
    uint8_t old_type = contents_type;
    set_contents_type_bits(new_type);
    
    if (new_type == MIXED_AS_LINES) {
      logical_node_lines_collection*& lpr = lines_pointer_reference();
      assert(lpr == NULL);
      lpr = new logical_node_lines_collection(bounds());
    }
    if (new_type == MIXED_AS_CHILDREN) {
      create_children((old_type == ALL_BLOCKED) ? ALL_BLOCKED : ALL_CLEAR);
    }
  }
}
/*
void logical_node::overwrite_from_other_collection(logical_node const& other) {
  set_contents_type(other.contents_type, false);
  if (other.contents_type == MIXED_AS_CHILDREN) {
    for (int which_child = 0; which_child < 4; ++which_child) {
      child(which_child).overwrite_from_other_collection(other.child(which_child));
    }
  }
  else if (other.contents_type == MIXED_AS_LINES) {
    (*(lines_pointer_reference())) = (*(other.lines_pointer_reference()));
    Maybe something other than =?
  }
}

bool logical_node::insert_from_other_collection(logical_node const& other) {
  assert(x_lower_bound_numerator == other.x_lower_bound_numerator);
  assert(y_lower_bound_numerator == other.y_lower_bound_numerator);
  assert(bounds_denominator == other.bounds_denominator);
  
  if (other.contents_type == ALL_CLEAR) {
    return false;
  }
  // Now: other contains some blockage.
  else if (contents_type == ALL_BLOCKED) {
    return false;
  }
  // Now: other contains some blockage and we contain some clear area.
  else if (contents_type == ALL_CLEAR) {
    overwrite_from_other_collection(other);
    // If we were all clear, then wherever their blockage is, it was visible.
    return true;
  }
  // Now: We are mixed and the other contains some blockage.
  else if (other.contents_type == ALL_BLOCKED) {
    set_contents_type(ALL_BLOCKED);
    // If they were all blocked, then wherever our clear area was, you could see them through it.
    return true;
  }
  // Now: Both ourselves and the other are mixed.
  else if ((contents_type == MIXED_AS_CHILDREN) && (other.contents_type == MIXED_AS_CHILDREN)) {
    bool result = false;
    for (int which_child = 0; which_child < 4; ++which_child) { 
      result = child(which_child).insert_from_other_collection(other.child(which_child)) || result;
    }
    return result;
  }
  else if ((contents_type == MIXED_AS_LINES) && (other.contents_type == MIXED_AS_LINES)) {
    
  }
  // Now the awkward cases where one of us has children and the other has lines.
  else if ((contents_type == MIXED_AS_LINES) && (other.contents_type == MIXED_AS_CHILDREN)) {
  }
  else if ((contents_type == MIXED_AS_CHILDREN) && (other.contents_type == MIXED_AS_LINES)) {
  }
  else {
    assert(false);
  }
}
*/

bool logical_node::lines_collection_is_visible(logical_node_lines_collection const& lines) {
  assert(lines.bounds == bounds());
  if (contents_type == MIXED_AS_LINES) { assert(lines_pointer_reference()->bounds == bounds()); }
  
  if (lines.empty()) {
    // Nothing there at all, so nothing there is visible.
    return false;
  }
  else if (contents_type == ALL_BLOCKED) {
    // We block everything, so nothing there is visible.
    return false;
  }
  else if (lines.full()) {
    // If they were full, then wherever our clear area was, you could see them through it.
    return true;
  }
  else if (contents_type == ALL_CLEAR) {
    // If we were all clear, then wherever their blockage is, it was visible.
    return true;
  }
  // Now: Both ourselves and the other are mixed.
  else if (contents_type == MIXED_AS_CHILDREN) {
    for (int which_child = 0; which_child < 4; ++which_child) {
      logical_node c = child(which_child);
      logical_node_lines_collection sub_lines(lines, c.bounds());
      if (c.lines_collection_is_visible(sub_lines)) return true;
    }
    return false;
  }
  else if (contents_type == MIXED_AS_LINES) {
    logical_node_lines_collection& l = *lines_pointer_reference();
    return l.allows_to_be_visible(lines);
  }
  else {
    assert(false);
  }
}

bool logical_node::insert_from_lines_collection(logical_node_lines_collection const& lines) {
  assert(lines.bounds == bounds());
  if (contents_type == MIXED_AS_LINES) { assert(lines_pointer_reference()->bounds == bounds()); }
  
  if (lines.empty()) {
    // Nothing there at all, so nothing there is visible.
    return false;
  }
  // Now: "lines" contains some blockage.
  else if (contents_type == ALL_BLOCKED) {
    // We block everything, so nothing there is visible.
    return false;
  }
  // Now: "lines" contains some blockage and we contain some clear area.
  else if (lines.full()) {
    set_contents_type(ALL_BLOCKED);
    // If they were full, then wherever our clear area was, you could see them through it.
    return true;
  }
  // Now: "lines" is mixed and we contain some clear area.
  else if (contents_type == ALL_CLEAR) {
    set_contents_type(MIXED_AS_LINES);
    lines_pointer_reference()->copy_from_same_or_child(lines);
    split_if_necessary();
    // If we were all clear, then wherever their blockage is, it was visible.
    return true;
  }
  // Now: Both ourselves and the other are mixed.
  else if (contents_type == MIXED_AS_CHILDREN) {
    bool result = false;
    // We can collapse the node if it ends up all blocked.
    // There are conceivable situations in which we could collapse the node
    // if some children were MIXED_AS_LINES, but it's a lot more complicated
    // to do that and may not actually be helpful in practice (it might take
    // more time than it saves)
    bool all_blocked_now = true;
    for (int which_child = 0; which_child < 4; ++which_child) {
      logical_node c = child(which_child);
      logical_node_lines_collection sub_lines(lines, c.bounds());
      if (c.insert_from_lines_collection(sub_lines)) result = true;
      if (c.contents_type != ALL_BLOCKED) all_blocked_now = false;
    }
    if (all_blocked_now) {
      set_contents_type(ALL_BLOCKED);
    }
    return result;
  }
  else if (contents_type == MIXED_AS_LINES) {
    logical_node_lines_collection& l = *lines_pointer_reference();
    const bool result = l.combine(lines);
    if (l.full()) {
      set_contents_type(ALL_BLOCKED);
    }
    else {
      split_if_necessary();
    }
    return result;
  }
  else {
    assert(false);
  }
}




} /* namespace collection_of_obscured_area_boundaries */

