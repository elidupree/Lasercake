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

#ifndef LASERCAKE_TIME_STEWARD_HPP__
#define LASERCAKE_TIME_STEWARD_HPP__

#include "siphash_id.hpp"

namespace time_steward_system {

namespace implementation {

template <typename physics_list, typename time_traits = default_time_traits>
struct physics {

};

//Hack: I copied the standard library implementation of upper/lower_bound,
//in order to search a range by a different type than the type in the range,
//such as searching by time in a range of objects that merely HAVE a time,
//which the standard doesn't allow
// (even though this implementation implements that case just fine).
template<typename _ForwardIterator, typename _Tp, typename _Compare>
    _ForwardIterator
    upper_bound (_ForwardIterator __first, _ForwardIterator __last,
		  const _Tp& __val, _Compare __comp)
    {
      typedef typename iterator_traits<_ForwardIterator>::difference_type
	_DistanceType;

      _DistanceType __len = std::distance(__first, __last);

      while (__len > 0)
	{
	  _DistanceType __half = __len >> 1;
	  _ForwardIterator __middle = __first;
	  std::advance(__middle, __half);
	  if (__comp(__val, __middle))
	    __len = __half;
	  else
	    {
	      __first = __middle;
	      ++__first;
	      __len = __len - __half - 1;
	    }
	}
      return __first;
    }
 template<typename _ForwardIterator, typename _Tp, typename _Compare>
    _ForwardIterator
    lower_bound(_ForwardIterator __first, _ForwardIterator __last,
		  const _Tp& __val, _Compare __comp)
    {
      typedef typename iterator_traits<_ForwardIterator>::difference_type
	_DistanceType;

      _DistanceType __len = std::distance(__first, __last);

      while (__len > 0)
	{
	  _DistanceType __half = __len >> 1;
	  _ForwardIterator __middle = __first;
	  std::advance(__middle, __half);
	  if (__comp(__middle, __val))
	    {
	      __first = __middle;
	      ++__first;
	      __len = __len - __half - 1;
	    }
	  else
	    __len = __half;
	}
      return __first;
    }
    
enum auditing_level_type {
  NONE,
  CONSTANT_FACTOR,
  HEAVY_DUTY,
  EXCESSIVE
};
template <typename physics, auditing_level_type auditing_level>
class time_steward {
  
  typedef std:: function <void (event_accessor &)> event_function;
  
  template <typename field_data>
  struct field_history {
    typedef std:: pair <extended_time, field_data> change;
    std:: vector <change> changes;
    std:: vector <extended_time> existence_changes;
    bool exists_before (extended_time time) {
      if (existence_changes.empty () || existence_changes [0] <time) {return false;}
      //optimize for checking at the end
      if (existence_changes.back () <time) {return existence_changes.size () & size_t (1);}
      auto after_most_recent = lower_bound (existence_changes.begin (), existence_changes.end (), time);
      return (after_most_recent - existence_changes.begin ()) & 1;
    }
    field_data const & state_before (extended_time time) {
      if (auditing_level >NONE) {
        caller_error_if (!exists_before (time), "accessing a field when it doesn't exist");
      }
      //optimize for checking at the end
      if (changes.back ().first <time) {return changes.back ().second;}
      return (lower_bound (changes.begin (), changes.end (), time,
        [] (extended_time const & alpha, change const & beta) {return alpha <beta.first;}) - 1)->second;
    }
  };
  template <typename field_data>
  struct field_history_map {
    std:: unordered_map <entity_ID, field_history> data;
  };
  physics:: data_for_each_field <field_history_map> field_histories;
  
  template <typename field_identifier, typename History>
  void erase_field_changes_since (History & history, extended_time time) {
    while (!history.changes.empty () && history.changes.back ().first >= time) {
      history.changes.pop_back ();
    }
    //theoretically, the "delete existence changes" part could be optimized
    //by occuring in code that is not duplicated for every field
    while (!history.existence_changes.empty () && history.existence_changes.back ().first >= time) {
      history.existence_changes.pop_back ();
    }
  }
  template <typename field_identifier>
  void transfer_field_change (event_accessor & accessor, entity_ID ID) {
    auto & history = field_histories.get <field_identifier> ();
    bool exists_before = history.exists_before (accessor.now ());
    bool exists_after =accessor.exists <field_identifier> (ID);
    if (!exists_before &&!exists_after) {return false;}
    //here would be the place to optimize by returning if the
    //DATA before and after were equal, if they are equality comparable
    erase_field_changes_since <field_identifier> (history, accessor.now ());
    if (exists_after) {
      history.changes.emplace_back (std:: move (accessor.entities.get <field_identifier> ()));
      if (!exists_before) {
        history.existence_changes.push_back (accessor.now ());
      }
    }
    else {
      assert (exists_before);
      history.existence_changes.push_back (accessor.now ());
    }
    return true;
  }
  template <typename field_identifier>
  void undo_field_change (extended_time time, entity_ID ID) {
    auto & history = field_histories.get <field_identifier> ();
    erase_field_changes_since <field_identifier> (history, time);
  }

  //we want these to be static members, but we can't figure out how to
  //make static members of templated classes work right
  /* static*/ const physics:: field_function_array <transfer_field_change> transfer_field_change_functions;
  /* static*/ const physics:: field_function_array <undo_field_change> undo_field_change_functions;
  
  void invalidate_accesses (extended_time time, field_ID ID) {
    
  }
  
  void do_event (event_reference event) {
    event_accessor accessor (this, event->time);
    event->function () (accessor);
    for (auto accessed: accessor.accessed) {
      record_access (event, accessed);
    }
    event->modified = std:: move (accessor.modified);
    std:: remove_if (event->modified.begin (), event->modified.end (),
      [] (field_ID const & ID) {
      if ((*transfer_field_change_functions [ID.field_index]) (accessor, ID.entity)) {
        invalidate_accesses (event->time, ID);
        return false;
      }
      return true;
    });
  }
  void undo_event (event_reference event) {
    for (auto ID: event->modified) {
      (*undo_field_change_functions [ID.field_index]) (event->time, ID.entity);
      invalidate_accesses (event->time, ID);
    }
  }
  void make_prediction (extended_time when, predictor_index predictor, entity_ID entity) {
    prediction_type prediction;
    prediction.made_at = when;
    predictor_accessor accessor (this, prediction);
    physics.run_predictor (predictor, accessor, entity);
    //that sets prediction.valid_until and prediction.prediction
    for (auto accessed: accessor.accessed) {
      record_access (prediction, accessed);
    }
    if (prediction.valid_until <max_time) {
      upcoming_issues.insert (MISSING_PREDICTION, predictor, prediction.valid_until);
    }
    if (prediction.prediction && next_event_time (prediction) <= prediction.valid_until) {
      upcoming_issues.insert (MISSING_PREDICTED_EVENT, prediction);
    }
  }
  void invalidate_prediction_after (prediction_reference prediction, extended_time time) {
    assert (time <prediction.valid_until);
    prediction->valid_until = time;
    while (!prediction->events.empty () && prediction->events.back ().time >time) {
      invalidate_event (prediction->events.back ());
      prediction->events.pop_back ();
    }
    if (prediction->valid_until >= prediction->made_at) {
      upcoming_issues.insert (MISSING_PREDICTION, prediction.predictor (), prediction->valid_until);
    }
    if (prediction->valid_until <= prediction->made_at) {
      prediction.destroy ();
    }
  }
  void invalidate_event (event*event) {
    upcoming_issues.insert (INVALID_EVENT, event);
  }
  void handle_issue (issue_type issue) {
    switch (issue.type) {
      case MISSING_PREDICTION: {
        
      } break;
    }
  }
  
public:
  void insert_fiat_event (time_type time, uint64_t distinguisher, event_function event) {
  
  }
};

}//End namespace implementation

using implementation:: time_steward;
using implementation:: physics_list;

}//end namespace time_steward_system

#endif
