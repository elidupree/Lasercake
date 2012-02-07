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

#ifndef LASERCAKE_BBOX_COLLISION_DETECTOR_HPP__
#define LASERCAKE_BBOX_COLLISION_DETECTOR_HPP__

#include <boost/integer.hpp>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <cassert>
#include <cstdlib>
#include <boost/scoped_ptr.hpp>
#include "utils.hpp"

using std::unordered_map;
using std::unordered_set;

typedef size_t num_bits_type;
typedef size_t num_coordinates_type;
typedef ptrdiff_t signed_num_coordinates_type;

// ObjectIdentifier needs hash and == and to be freely copiable. So, ints will do, pointers will do...
// CoordinateBits should usually be 32 or 64. I don't know if it works for other values.
template<typename ObjectIdentifier, num_bits_type CoordinateBits, num_coordinates_type NumDimensions>
class bbox_collision_detector {
  static_assert(NumDimensions >= 0, "You can't make a space with negative dimensions!");
  static_assert(CoordinateBits >= 0, "You can't have an int type with negative bits!");
  friend class zbox_tester;

public:
  typedef typename boost::uint_t<CoordinateBits>::fast Coordinate;
  struct bounding_box {
    std::array<Coordinate, NumDimensions> min, size;
    bool overlaps(bounding_box const& other)const {
      for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
        // this should correctly handle zboxes' "size=0" when all bits are ignored
        if (other.min[i] + (other.size[i] - 1) <       min[i]) return false;
        if (      min[i] + (      size[i] - 1) < other.min[i]) return false;
      }
      return true;
    }
  };
  
  bbox_collision_detector():objects_tree(nullptr){}
  bbox_collision_detector(bbox_collision_detector const& other) { *this = other; }
  bbox_collision_detector& operator=(bbox_collision_detector const& other) {
    bboxes_by_object = other.bboxes_by_object;
    if(other.objects_tree) objects_tree.reset(new ztree_node(*other.objects_tree));
    return *this;
  }
  
private:
  static const num_bits_type total_bits = CoordinateBits * NumDimensions;
  
  static Coordinate safe_left_shift_one(num_bits_type shift) {
    if (shift >= 8*sizeof(Coordinate)) return 0;
    return Coordinate(1) << shift; // TODO address the fact that this could put bits higher than the appropriate amount if CoordinateBits isn't the number of bits of the type
  }
  
  struct zbox {
    // We ensure that every bit except the ones specifically supposed to be on is off.
    std::array<Coordinate, NumDimensions> coords;
    std::array<Coordinate, NumDimensions> interleaved_bits;
    num_bits_type num_low_bits_ignored;
    bounding_box bbox;
    
    zbox():num_low_bits_ignored(total_bits){ for (num_coordinates_type i = 0; i < NumDimensions; ++i) interleaved_bits[i] = 0; }
    
    bool subsumes(zbox const& other)const {
      if (other.num_low_bits_ignored > num_low_bits_ignored) return false;
      for (num_coordinates_type i = num_low_bits_ignored / CoordinateBits; i < NumDimensions; ++i) {
        Coordinate mask = ~Coordinate(0);
        if (i == num_low_bits_ignored / CoordinateBits) {
          mask &= ~(safe_left_shift_one(num_low_bits_ignored % CoordinateBits) - 1);
        }
        if ((interleaved_bits[i] & mask) != (other.interleaved_bits[i] & mask)) return false;
      }
      return true;
    }
    bool get_bit(num_bits_type bit)const {
      return interleaved_bits[bit / CoordinateBits] & safe_left_shift_one(bit % CoordinateBits);
    }
    num_bits_type num_bits_ignored_by_dimension(num_coordinates_type dim)const {
      return (num_low_bits_ignored + (NumDimensions - 1) - dim) / NumDimensions;
    }
    // note: gives "size=0" for max-sized things
    void compute_bbox() {
      bbox.min = coords;
      for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
        bbox.size[i] = safe_left_shift_one(num_bits_ignored_by_dimension(i));
      }
    }
    bounding_box const& get_bbox()const {
      return bbox;
    }
  };
  
  static int idx_of_highest_bit(Coordinate i) {
    int upper_bound = CoordinateBits;
    int lower_bound = -1;
    while(true) {
      int halfway_bit_idx = (upper_bound + lower_bound) >> 1;
      if (halfway_bit_idx == lower_bound) return lower_bound;
      
      if (i & ~(safe_left_shift_one(halfway_bit_idx) - 1)) lower_bound = halfway_bit_idx;
      else                                                 upper_bound = halfway_bit_idx;
    }
  }
  
  struct ztree_node;
  typedef boost::scoped_ptr<ztree_node> ztree_node_ptr;
  struct ztree_node {
    zbox here;
    ztree_node_ptr child0;
    ztree_node_ptr child1;
    unordered_set<ObjectIdentifier> objects_here;
    
    ztree_node(zbox box):here(box),child0(nullptr),child1(nullptr){}
    ztree_node(ztree_node const& other) { *this = other; }
    ztree_node& operator=(ztree_node const& other) {
      here = other.here;
      if(other.child0) child0.reset(new ztree_node(*other.child0));
      if(other.child1) child1.reset(new ztree_node(*other.child1));
      return *this;
    }
  };
  
  // TODO: Can we make these functions be constructors of zbox,
  // and make zbox's members private? zbox could REALLY use some
  // data hiding (right now it's pretty clear-cut who's allowed to
  // edit zbox data, but it isn't enforced.)
  static zbox smallest_joint_parent(zbox zb1, zbox zb2) {
    zbox new_box;
    const num_bits_type max_ignored = std::max(zb1.num_low_bits_ignored, zb2.num_low_bits_ignored);
    for (signed_num_coordinates_type i = NumDimensions - 1; i >= 0; --i) {
      int highest_bit_idx = idx_of_highest_bit(zb1.interleaved_bits[i] ^ zb2.interleaved_bits[i]);
      if ((highest_bit_idx+1 + i * CoordinateBits) < max_ignored) highest_bit_idx = max_ignored - i * CoordinateBits - 1;
#ifdef ASSERT_EVERYTHING
      assert((zb1.interleaved_bits[i] & ~((safe_left_shift_one(highest_bit_idx+1)) - 1)) == (zb2.interleaved_bits[i] & ~((safe_left_shift_one(highest_bit_idx+1)) - 1)));
#endif
      new_box.interleaved_bits[i] = (zb1.interleaved_bits[i]) & (~((safe_left_shift_one(highest_bit_idx + 1)) - 1));
      if (highest_bit_idx >= 0) {
        new_box.num_low_bits_ignored = highest_bit_idx+1 + i * CoordinateBits;
        for (num_coordinates_type j = 0; j < NumDimensions; ++j) {
#ifdef ASSERT_EVERYTHING
          assert(             (zb1.coords[j] & ~(safe_left_shift_one(new_box.num_bits_ignored_by_dimension(j)) - 1))
                           == (zb2.coords[j] & ~(safe_left_shift_one(new_box.num_bits_ignored_by_dimension(j)) - 1)));
#endif
          new_box.coords[j] = zb1.coords[j] & ~(safe_left_shift_one(new_box.num_bits_ignored_by_dimension(j)) - 1);
        }
        new_box.compute_bbox();
        return new_box;
      }
    }
    new_box.num_low_bits_ignored = max_ignored;
#ifdef ASSERT_EVERYTHING
    assert(zb1.coords == zb2.coords);
#endif
    new_box.coords = zb1.coords;
    new_box.compute_bbox();
    return new_box;
  }
  
  static zbox box_from_coords(std::array<Coordinate, NumDimensions> const& coords, num_bits_type num_low_bits_ignored) {
    zbox result;
    result.num_low_bits_ignored = num_low_bits_ignored;
    for (num_coordinates_type i = 0; i < NumDimensions; ++i) {
      result.coords[i] = coords[i] & (~(safe_left_shift_one(result.num_bits_ignored_by_dimension(i)) - 1));
    }
    for (num_bits_type bit_within_interleaved_bits = num_low_bits_ignored;
                       bit_within_interleaved_bits < total_bits;
                     ++bit_within_interleaved_bits) {
      const num_bits_type bit_idx_within_coordinates = bit_within_interleaved_bits / NumDimensions;
      const num_coordinates_type which_coordinate    = bit_within_interleaved_bits % NumDimensions;
      const num_bits_type interleaved_bit_array_idx  = bit_within_interleaved_bits / CoordinateBits;
      const num_bits_type interleaved_bit_local_idx  = bit_within_interleaved_bits % CoordinateBits;
#ifdef ASSERT_EVERYTHING
      assert(bit_idx_within_coordinates >= result.num_bits_ignored_by_dimension(which_coordinate));
#endif
      result.interleaved_bits[interleaved_bit_array_idx] |= ((coords[which_coordinate] >> bit_idx_within_coordinates) & 1) << interleaved_bit_local_idx;
    }
    result.compute_bbox();
    return result;
  }
  
  static void insert_box(ztree_node_ptr& tree, ObjectIdentifier const& obj, zbox box) {
    if (!tree) {
      tree.reset(new ztree_node(box));
      tree->objects_here.insert(obj);
    }
    else {
      if (tree->here.subsumes(box)) {
        if (box.num_low_bits_ignored == tree->here.num_low_bits_ignored) {
          tree->objects_here.insert(obj);
        }
        else {
          if (box.get_bit(tree->here.num_low_bits_ignored - 1)) insert_box(tree->child1, obj, box);
          else                                                  insert_box(tree->child0, obj, box);
        }
      }
      else {
        ztree_node_ptr new_tree(new ztree_node(smallest_joint_parent(tree->here, box)));

#ifdef ASSERT_EVERYTHING
        assert(new_tree->here.num_low_bits_ignored > tree->here.num_low_bits_ignored);
        assert(new_tree->here.subsumes(tree->here));
        assert(new_tree->here.subsumes(box));
        assert(box.subsumes(tree->here) || (tree->here.get_bit(new_tree->here.num_low_bits_ignored - 1) != box.get_bit(new_tree->here.num_low_bits_ignored - 1)));
#endif

        if (tree->here.get_bit(new_tree->here.num_low_bits_ignored - 1)) tree.swap(new_tree->child1);
        else                                                             tree.swap(new_tree->child0);

        tree.swap(new_tree);
        insert_box(tree, obj, box);
      }
    }
  }
  
  static void delete_object(ztree_node_ptr& tree, ObjectIdentifier const& obj, bounding_box const& bbox) {
    if (!tree) return;
    if (tree->here.get_bbox().overlaps(bbox)) {
      tree->objects_here.erase(obj);
      delete_object(tree->child0, obj, bbox);
      delete_object(tree->child1, obj, bbox);
      
      if (tree->objects_here.empty()) {
        if (tree->child0) {
          if (!tree->child1) {
            ztree_node_ptr dead_tree;
            dead_tree.swap(tree);
            dead_tree->child0.swap(tree);
          }
        }
        else {
          // old 'child1' a.k.a. new 'tree' could be null
          ztree_node_ptr dead_tree;
          dead_tree.swap(tree);
          dead_tree->child1.swap(tree);
        }
      }
    }
  }
  
  void zget_objects_overlapping(ztree_node const* tree, unordered_set<ObjectIdentifier>& results, bounding_box const& bbox)const {
    if (tree && tree->here.get_bbox().overlaps(bbox)) {
      for (const ObjectIdentifier obj : tree->objects_here) {
        auto bbox_iter = bboxes_by_object.find(obj);      
#ifdef ASSERT_EVERYTHING
        assert(bbox_iter != bboxes_by_object.end());
#endif
        if (bbox_iter->second.overlaps(bbox)) results.insert(obj);
      }
      zget_objects_overlapping(tree->child0.get(), results, bbox);
      zget_objects_overlapping(tree->child1.get(), results, bbox);
    }
  }
  
  unordered_map<ObjectIdentifier, bounding_box> bboxes_by_object;
  ztree_node_ptr objects_tree;
  
public:

  void insert(ObjectIdentifier const& id, bounding_box const& bbox) {
    caller_correct_if(
      bboxes_by_object.insert(std::make_pair(id, bbox)).second,
      "bbox_collision_detector::insert() requires for your safety that the id "
      "is not already in this container.  Use .erase() or .exists() if you need "
      "any particular behaviour in this case."
    );
    Coordinate max_dim = bbox.size[0];
    for (num_coordinates_type i = 1; i < NumDimensions; ++i) {
      if (bbox.size[i] > max_dim) max_dim = bbox.size[i];
    }
    int exp = 0; while ((Coordinate(1) << exp) < max_dim) ++exp;
    int dimensions_we_can_single = 0;
    int dimensions_we_can_double = 0;
    const Coordinate base_box_size = safe_left_shift_one(exp);
    const Coordinate used_bits_mask = ~(base_box_size - 1);
    
    for (int i = NumDimensions - 1; i >= 0; --i) {
      if ((bbox.min[i] & used_bits_mask) + base_box_size >= bbox.min[i] + bbox.size[i]) ++dimensions_we_can_single;
      else break;
    }
    for (num_coordinates_type i = 0; i < NumDimensions - dimensions_we_can_single; ++i) {
      if (!(bbox.min[i] & base_box_size)) ++dimensions_we_can_double;
      else break;
    }
    #ifdef ZTREE_TESTING
    std::cerr << dimensions_we_can_single << "... " << dimensions_we_can_double << "...\n";
    #endif
    for (int i = 0; i < (1 << ((NumDimensions - dimensions_we_can_single) - dimensions_we_can_double)); ++i) {
      std::array<Coordinate, NumDimensions> coords = bbox.min;
      for (num_coordinates_type j = dimensions_we_can_double; j < NumDimensions - dimensions_we_can_single; ++j) {
        if ((i << dimensions_we_can_double) & (1<<j)) coords[j] += base_box_size;
      }
      zbox zb = box_from_coords(coords, exp * NumDimensions + dimensions_we_can_double);
      if (zb.get_bbox().overlaps(bbox))
        insert_box(objects_tree, id, zb);
    }
  }
  
  bool exists(ObjectIdentifier const& id)const {
    return (bboxes_by_object.find(id) != bboxes_by_object.end());
  }
  
  bool erase(ObjectIdentifier const& id) {
    auto bbox_iter = bboxes_by_object.find(id);
    if (bbox_iter == bboxes_by_object.end()) return false;
    delete_object(objects_tree, id, bbox_iter->second);
    bboxes_by_object.erase(bbox_iter);
    return true;
  }

  bounding_box const* find_bounding_box(ObjectIdentifier const& id)const {
    return find_as_pointer(bboxes_by_object, id);
  }
  
  void get_objects_overlapping(unordered_set<ObjectIdentifier>& results, bounding_box const& bbox)const {
    zget_objects_overlapping(objects_tree.get(), results, bbox);
  }
  
private:
  struct generalized_object_collection_walker;

public:
  class generalized_object_collection_handler {
  public:
    virtual void handle_new_find(ObjectIdentifier) {}
    
    // Any bbox for which this one of these functions returns false is ignored.
    // The static one is checked only once for each box; the dynamic one is checked
    // both before the box is added to the frontier and when it comes up.
    virtual bool should_be_considered__static(bounding_box const&)const { return true; }
    virtual bool should_be_considered__dynamic(bounding_box const&)const { return true; }
    
    // returns "true" if the first one should be considered first, "false" otherwise.
    // This is primarily intended to allow the caller to run through objects in a specific
    // in-space order to get the first collision in some direction. One can combine it with
    // a dynamic should_be_considered function to stop looking in boxes that are entirely
    // after the first known collision.
    //
    // This function must be a Strict Weak Ordering and not change its behavior.
    // This function is only a heuristic; we don't guarantee that the boxes will be handled
    // exactly in the given order.
    virtual bool bbox_ordering(bounding_box const&, bounding_box const&)const { return false; } // arbitrary order
    
    // This is called often (specifically, every time we call a non-const function of handler);
    // if it returns true, we stop right away.
    // Some handlers might know they're done looking before the should_be_considered boxes
    // run out; this just makes that more efficient.
    virtual bool done()const { return false; }
    
    unordered_set<ObjectIdentifier> const& get_found_objects()const { return found_objects; }
  private:
    unordered_set<ObjectIdentifier> found_objects;
    friend struct generalized_object_collection_walker;
  };

private:
  struct generalized_object_collection_walker {
    generalized_object_collection_handler *handler;
    unordered_map<ObjectIdentifier, bounding_box> const& bboxes_by_object;
    generalized_object_collection_walker(generalized_object_collection_handler *handler, unordered_map<ObjectIdentifier, bounding_box> const& bboxes_by_object):handler(handler),bboxes_by_object(bboxes_by_object),frontier(set_compare(handler)){}
    
    struct set_compare {
      generalized_object_collection_handler *handler;
      set_compare(generalized_object_collection_handler *handler):handler(handler){}
      
      bool operator()(ztree_node const* z1, ztree_node const* z2)const {
        return handler->bbox_ordering(z1->here.bbox, z2->here.bbox);
      }
    };
    
    // will only contain nonnull pointers (since it would be a waste of time to
    // insert null pointers into the set).
    //
    // TODO: we can probably find a more optimal data structure to use here than std::set.
    std::set<ztree_node const*, set_compare> frontier;
    
    void try_add(ztree_node* z) {
      if (z && handler->should_be_considered__static(z->here.bbox) && handler->should_be_considered__dynamic(z->here.bbox)) {
        frontier.insert(z);
      }
    }
      
    bool process_next() {
      if (frontier.empty()) return false;
      
      ztree_node const* next = *frontier.begin();
      frontier.erase(frontier.begin());
      
      if (handler->should_be_considered__dynamic(next->here.bbox)) {
        for (const ObjectIdentifier obj : next->objects_here) {
          auto bbox_iter = bboxes_by_object.find(obj);
#ifdef ASSERT_EVERYTHING
          assert(bbox_iter != bboxes_by_object.end());
#endif
          if (handler->should_be_considered__static(bbox_iter->second) && handler->should_be_considered__dynamic(bbox_iter->second)) {
            if (handler->found_objects.insert(obj).second) {
              handler->handle_new_find(obj);
              if (handler->done()) return false;
            }
          }
        }
        try_add(next->child0.get());
        try_add(next->child1.get());
      }
      
      return true;
    }
  };
    
public:
  void get_objects_generalized(generalized_object_collection_handler* handler)const {
    generalized_object_collection_walker data(handler, bboxes_by_object);
    
    data.try_add(objects_tree.get());
    while(data.process_next()){}
  }
};

/*

If there's one zbox in the tree [call it Z]

tree = ztree_node {
  Z
  nullptr
  nullptr
}

Two zboxes that differ at bit B:
child0 of a node with B ignored bits is the child whose Bth bit is 0.
tree = ztree_node {
  Z1/Z2, with B ignored bits
  ptr to ztree_node {
    Z1
    nullptr
    nullptr
  }
  ptr to ztree_node {
    Z2
    nullptr
    nullptr
  }
}

*/

#endif

