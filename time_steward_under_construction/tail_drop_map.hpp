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

#ifndef LASERCAKE_TAIL_DROP_MAP_HPP__
#define LASERCAKE_TAIL_DROP_MAP_HPP__

/*

This is a bizarrely specific data structure that might be useful for the time steward.

It implements only two operations:
void insert (key, value), which inserts an element into the map, and
tail_drop (key), which removes and returns all elements with keys after key, not necessarily in order.

Inserting at the end is O(1) worst case.
Inserting elsewhere is O(1), but incurs a debt of up to O(log n) which will be paid off later by tail_drop.
tail_drop is merely O(number of things returned), plus paying off the debt.

Since the results of tail_drop don't actually need to be sorted,
we delay all or part of the O(log n) cost of sorting for as long as we can,
in the hopes of never having to pay it at all.

There is also discard_head (key), which can be used to save memory by assuring the tail_drop_map that you won't query before key, so it can discard some data that it knows will never be used. However, it does not guarantee that all data before key will be discarded.

*/

template <typename key_type, typename mapped_type>
class tail_drop_map {
private:
  //the "bucket" type is used only for collecting unsorted elements
  //1 at a time, then eventually iterating them and dumping them all out.
  typedef std:: vector bucket;
  typedef std:: pair <key_type, mapped_type> value_type;
  
  struct sorted_element {
    sorted_element (value_type const & value): value (value) {}
    value_type value;
    bucket unsorted_elements_before_or_equal_this;
  };
  std:: vector <sorted_element> sorted_elements;
  bucket unsorted_elements;
  
public:
  void insert (value_type const & value) {
    if (value.first >= sorted_elements.back ().value.first) {
      sorted_elements.emplace_back (value);
    }
    else {
      unsorted_elements.push_back (value);
    }
  }
  
  template <class output_function>
  void tail_drop (key_type const & key, output_function output = output_function ()) {
    if (sorted_elements.empty ()) {
      assert (unsorted_elements.empty ());
      return;
    }
    while (sorted_elements.back ().value.first >key) {
      bucket & spilling = sorted_elements.back ().unsorted_elements_before_or_equal_this;
      std:: move (spilling.begin (), spilling.end (), std:: back_inserter (unsorted_elements));
      output (sorted_elements.back ().value);
      sorted_elements.pop_back ();
    }

    remaining_sorted= sorted_elements.size ();
    for (value_type value: unsorted_elements) {
      if (value.first >key) {
        output (value);
      }
      else if (sorted_elements.empty () ||
          value.first >= sorted_elements [remaining_sorted-1].value.first) {
        sorted_elements.emplace_back (value);
      }
      else {
        //do part of a binary search, but don't do more than you have to.
        //If there's never a tail_drop that goes back that far,
        //you never have to do the rest of the search, AND
        //if there IS a tail_drop that goes far enough back to
        //actually OUTPUT this element, we never have to do the rest
        //of the search in that scenario either.
        size_t dump_location = remaining_sorted>> 1;
        while (sorted_elements [dump_location].value.first < value.first) {
          dump_location = (dump_location + remaining_sorted) >> 1;
        }
        sorted_elements [dump_location].unsorted_elements_before_or_equal_this
          .push_back (value);
      }
    }
    unsorted_elements.clear ();
    if (sorted_elements.size () >remaining_sorted + 1) {
      std:: sort (sorted_elements [remaining_sorted], sorted_elements.end (),
        [] (value_1, value_2) {return value_1.first <value_2.first});
    }
  }
  
  void discard_head (key_type const & key) {
    //this could probably be improved
    if (sorted_elements.empty ()) { return;}
    auto middle = sorted_elements.begin () + (sorted_elements.size () >> 1);
    if (middle -> value.first <key) {
      sorted_elements.erase (sorted_elements.begin (), middle);
    }
  }
};

#endif
