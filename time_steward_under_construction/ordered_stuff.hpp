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


namespace ordered_stuff_system {
typedef uint64_t idx_type;
typedef uint32_t num_bits_type;
const idx_type no_idx = std::numeric_limits<idx_type>::max();

template<typename ValueType> class ordered_stuff;
namespace impl {
template<typename ValueType>
struct entry {
  template <class... Args>
  entry(idx_type idx, Args&&... args):idx(idx),ref_count(0),contents(std::forward<Args>(args)...){}
  idx_type idx;
  size_t ref_count;
  ValueType contents;
};
template<typename ValueType>
struct entry_ref {
public:
  entry_ref():data(nullptr){}
  entry_ref(entry_ref const& o):data(o.data) { if (data) { ++data->ref_count; } }
  bool operator< (entry_ref const& o)const { return data->idx <  o.data->idx; }
  bool operator> (entry_ref const& o)const { return data->idx >  o.data->idx; }
  bool operator<=(entry_ref const& o)const { return data->idx <= o.data->idx; }
  bool operator>=(entry_ref const& o)const { return data->idx >= o.data->idx; }
  bool operator==(entry_ref const& o)const { return data == o.data; }
  bool operator!=(entry_ref const& o)const { return data != o.data; }
  explicit operator bool()const { return bool(data); }
  ValueType& operator*()const { return data->contents; }
  ValueType* operator->()const { return &data->contents; }
  entry_ref& operator=(entry_ref const& o) {
    if (o.data) { ++o.data->ref_count; }
    dec_ref();
    data = o.data;
  }
  ~entry_ref() { dec_ref(); }
private:
  void dec_ref() {
    if (data && (--data->ref_count == 0)) {
      delete data;
    }
  }
  entry_ref(entry<ValueType>* data):data(data){ ++data->ref_count; }
  entry<ValueType>* data;
  friend class ordered_stuff<ValueType>;
  friend struct std::hash<entry_ref>;
};
}

//#define AUDIT_ORDERED_STUFF

template<typename ValueType>
class ordered_stuff {
  typedef impl::entry<ValueType> entry;
public:
  typedef impl::entry_ref<ValueType> entry_ref;
  template <class... Args>
  entry_ref construct(Args&&... args) {
    // TODO: better allocator
    return entry_ref(new entry(no_idx, std::forward<Args>(args)...));
  }

  // TODO: allow moving and deletion
  void put_only(entry_ref moving) {
    place_at(0, moving);
  }
  void put_before(entry_ref moving, entry_ref relative_to) {
#ifdef AUDIT_ORDERED_STUFF
    std::map<idx_type, entry_ref> old_data = data;
#endif
    const idx_type idx = relative_to.data->idx;
    std::pair<idx_type,idx_type> p = make_room_for_split(idx & ~3, 0, idx & 3);
    move_entry(idx, p.second);
    place_at(p.first, moving);
#ifdef AUDIT_ORDERED_STUFF
    assert(relative_to > moving);
    entry_ref prev;
    for (auto& f : old_data) {
      if (prev) assert(prev < f.second);
      if (f.second < relative_to) assert(f.second < moving);
      if (f.second > relative_to) assert(f.second > moving);
      prev = f.second;
      //std::cerr << f.second.data->idx << ", ";
    }
#endif
  }
  void put_after(entry_ref moving, entry_ref relative_to) {
#ifdef AUDIT_ORDERED_STUFF
    std::map<idx_type, entry_ref> old_data = data;
#endif
    const idx_type idx = relative_to.data->idx;
    std::pair<idx_type,idx_type> p = make_room_for_split(idx & ~3, 0, idx & 3);
    move_entry(idx, p.first);
    place_at(p.second, moving);
#ifdef AUDIT_ORDERED_STUFF
    assert(relative_to < moving);
    entry_ref prev;
    for (auto& f : old_data) {
      if (prev) assert(prev < f.second);
      if (f.second < relative_to) assert(f.second < moving);
      if (f.second > relative_to) assert(f.second > moving);
      prev = f.second;
      //std::cerr << f.second.data->idx << ", ";
    }
#endif
  }
private:
  // TODO: a custom hashtable for this so we don't need the extra layer of pointers
#ifdef AUDIT_ORDERED_STUFF
  std::map<idx_type, entry_ref> data;
#else
  std::unordered_map<idx_type, entry_ref> data;
#endif
  bool idx_exists(idx_type idx)const { return data.find(idx) != data.end(); }
  
  // The data forms - mathematically, but not in memory - a lenient B-tree with 2,3,or 4 children for each node.
  // When it overflows to 5, it splits into a 3-node and a 2-node.
  // An index's Nth bit-pair indicates which child it is at the Nth level of the tree (from the leaves).
  // Since a node's first child always exists, parent nodes (which have no data)
  // are represented by the ID of the first descendant leaf, plus their size/level.
  std::pair<idx_type,idx_type> make_room_for_split(idx_type prefix, num_bits_type child_shift, idx_type which_child) {
    const idx_type child_size = 1 << child_shift;
    if (idx_exists(prefix+3*child_size)) {
      // split this node to make room for splitting children
      const num_bits_type parent_shift = child_shift + 2;
      const idx_type parent_size = 1 << parent_shift;
      const idx_type next_which_child_mask = parent_size * 0x3;
      std::pair<idx_type,idx_type> p = make_room_for_split(prefix & ~next_which_child_mask, parent_shift, (prefix & next_which_child_mask) >> parent_shift);
      assert(p.first >= prefix);
      assert(p.second >= p.first + parent_size);
      assert(!idx_exists(p.second));
      
      // Don't move which_child, because the caller will move it.
      // Don't put anything into result.first or result.second.
      if (which_child != 3) { move_subtree_if_necessary(
        prefix + 3*child_size, child_shift, /*(which_child < 3) ?*/p.second + 1*child_size/*:  p.second            */); }
      if (which_child != 2) { move_subtree_if_necessary(
        prefix + 2*child_size, child_shift,   (which_child < 2) ?  p.second                 :  p.first + 2*child_size); }
      if (which_child != 1) { move_subtree_if_necessary(
        prefix + 1*child_size, child_shift,   (which_child < 1) ?  p.first  + 2*child_size  :  p.first + 1*child_size); }
      if (which_child != 0) { move_subtree_if_necessary(
        prefix               , child_shift, /*(which_child < 0) ?  p.first  + 1*child_size  :*/p.first               ); }
      
      if (which_child == 0) {
        return std::pair<idx_type,idx_type>(p.first               , p.first  + 1*child_size);
      }
      else if (which_child == 1) {
        return std::pair<idx_type,idx_type>(p.first + 1*child_size, p.first  + 2*child_size);
      }
      else if (which_child == 2) {
        return std::pair<idx_type,idx_type>(p.first + 2*child_size, p.second              );
      }
      else if (which_child == 3) {
        return std::pair<idx_type,idx_type>(p.second,               p.second + 1*child_size);
      }
      else {
        assert(false);
      }
    }

    // Make result.second empty. Don't move which_child, because the caller will handle it.
    assert (which_child < 3);
    if ((which_child < 2) && idx_exists(prefix + 2*child_size)) { move_subtree(prefix + 2*child_size, child_shift, prefix + 3*child_size); }
    if  (which_child < 1)                                       { move_subtree(prefix + 1*child_size, child_shift, prefix + 2*child_size); }
    return std::pair<idx_type,idx_type>(prefix + which_child*child_size, prefix + (which_child+1)*child_size);
  }
  void move_subtree_if_necessary(idx_type prefix, num_bits_type node_shift, idx_type new_prefix) {
    if (prefix != new_prefix) { move_subtree(prefix, node_shift, new_prefix); }
  }
  void move_subtree(idx_type prefix, num_bits_type node_shift, idx_type new_prefix) {
    if (node_shift == 0) {
      move_entry(prefix, new_prefix);
    }
    else {
      const num_bits_type child_shift = node_shift-2;
      const idx_type child_size = 1 << child_shift;
      for (int i = 3; i >= 0; --i) {
        const idx_type child_prefix = prefix + child_size*i;
        if (idx_exists(child_prefix)) {
          move_subtree(child_prefix, child_shift, new_prefix + child_size*i);
        }
      }
    }
  }
  void move_entry(idx_type idx, idx_type new_idx) {
    if (new_idx != idx) {
      auto i = data.find(idx);
      if (i != data.end()) {
        i->second.data->idx = new_idx;
        auto p = data.insert(std::make_pair(new_idx, i->second));
        assert(p.second);
        data.erase(i);
      }
    }
  }
  void place_at(idx_type idx, entry_ref moving) {
    assert(moving.data->idx == no_idx);
    moving.data->idx = idx;
    auto p = data.insert(std::make_pair(idx, moving));
    assert(p.second);
  }
};
} // namespace ordered_stuff_system
using ordered_stuff_system::ordered_stuff;
namespace std {
  template<typename ValueType>
  struct hash<typename ordered_stuff_system::impl::entry_ref<ValueType>> {
    public:
    size_t operator()(ordered_stuff_system::impl::entry_ref<ValueType> const& i)const {
      // Nondeterministic - but the ordering of STL hash tables is already nondeterministic.
      return std::hash<decltype(i.data)>()(i.data);
    }
  };
}