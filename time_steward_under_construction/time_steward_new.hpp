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
using intrusive_pointer = boost::intrusive_ptr
template <typename physics, auditing_level_type auditing_level>
class time_steward {
  
  typedef std:: function <void (event_accessor &)> event_function;
  typedef uint64_t serial_number_type;
  
  template <typename field_data>
  struct field_history {
    typedef std:: pair <intrusive_pointer <event_type>, field_data> change;
    std:: vector <change> changes;
    std:: vector <intrusive_pointer <event_type>> existence_changes;
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
  typedef intrusive_pointer <access_info> access_pointer;
  struct prediction_history;
  struct prediction_type: public boost::intrusive_ref_counter <prediction_type> {
    //null if this prediction is invalid
    prediction_history*history;
    //this could presumably be a regular pointer,
    //but for now, I'm just going to be extra memory-safe.
    intrusive_pointer <prediction_type> previous;
    extended_time made_at;
    extended_time valid_until;
    access_pointer latest_access;
    
    //do we need to reference-count predictions?
    //One possible model is that the "what accessed this" collections
    //contain one pointer for each entry, and in the case of predictions,
    //that pointer is a smart pointer that reference counts them.
    //Another model is that the data is not a pointer but a serial number,
    //which must be looked up in a hash table; in that case, we could just
    //delete the prediction from the hash table, and not have a reference count.
    //However, we would need a special datatype that could automatically ignore
    //values that have been deleted (because they have no ordering)
    
    time_type result_time;
    event_function result_function;
    intrusive_pointer <event_type> last_event;
  };
  struct prediction_history {
    predictor_index predictor;
    entity_ID entity;
    intrusive_pointer <prediction_type> last;
  }
  typedef std::vector <prediction_type> prediction_history;
  struct event_type: public boost::intrusive_ref_counter <event_type> {
    intrusive_pointer <event_type> previous_from_same_prediction;
    extended_time time;
    bool valid;
    std::vector <entity_ID> modified;
  };
  struct access_info: public boost::intrusive_ref_counter <access_info> {
    //the time variable is possibly redundant?
    //If we reference-count events, then we can just use event->time.
    //(Right now, event is null if this was an access by a prediction.)
    //For the initial implementation, though, we don't care about optimizing the
    //space usage of this struct.
    extended_time time;
    intrusive_pointer <event_type> event;
    intrusive_pointer <prediction_type> prediction;
  };
  struct access_history {
    struct sort {bool operator< (access_pointer const & alpha, access_pointer const & beta) const {
      //standard ordering, because we actually do want the highest ones first
      return alpha->time <beta->time;
    }};
    std::priority_queue <access_pointer, std::vector <access_pointer>, sort> bounded_accesses;
    std::vector <access_pointer> current_prediction_accesses;
  };
  
  serial_number_type next_serial_number = 0;
  physics:: data_for_each_field <field_history_map> field_histories;
  std::unordered_map <prediction_history_ID, prediction_history> prediction_histories;
  std::unordered_map <field_ID, access_history> access_histories;
  std::unordered_map <serial_number_type, prediction_type> predictions;
  std::unordered_map <serial_number_type, event_type> events;
  issue_collector upcoming_issues;
  
  template <typename field_identifier, typename History>
  void erase_field_changes_since (History & history, extended_time time) {
    while (!history.changes.empty () && history.changes.back ().first->time >= time) {
      invalidate_event_results (*history.changes.back ().first);
      history.changes.pop_back ();
    }
    //theoretically, the "delete existence changes" part could be optimized
    //by occuring in code that is not duplicated for every field
    while (!history.existence_changes.empty () && history.existence_changes.back ()->time >= time) {
      invalidate_event_results (*history.existence_changes.back ());
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
        physics:: for_predictors_watching <field_identifier> ([& ID, & accessor] (predictor_index predictor) {
          upcoming_issues.insert (MISSING_PREDICTION, predictor, accessor.now ());
        });
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
  
  void invalidate_accesses_since (extended_time time, field_ID ID, bool undoing_event = false) {
    auto iterator = access_histories.find (ID);
    if (iterator == access_histories.end ()) return;
    invalidate_accesses_since (time, iterator->second, undoing_event);
  }
  void invalidate_accesses_since (extended_time time, access_history & history, bool undoing_event) {
    for (access_pointer const & access: history.current_prediction_accesses) {
      invalidate_access (time, history, access);
    }
    history.current_prediction_accesses.clear ();
    while (history.bounded_accesses.top ()->time >time) {
      invalidate_access (time, history, history.bounded_accesses.top (), undoing_event);
      history.bounded_accesses.pop ();
    }
  }
  void invalidate_access (extended_time time, access_history & history, access_pointer const & access, bool undoing_event) {
    if (access->prediction) {
      auto & history =*access->prediction->history;
      bool needs_replacement = access->prediction.made_at <time;
      if (time <access->prediction->valid_until) {
        for (; history.last != access->prediction; history.last = history.last->previous) {
          assert (history.last->made_at >= time);
          invalidate_prediction (*history.last);
        }
        invalidate_prediction_after (*access->prediction, time);
      }
      if (needs_replacement) {
        history.bounded_accesses.push (prediction.latest_access);
        //the above may invalidate access.
        //Therefore, we can't use access after this. Just be safe,
        return;
      }
    }
    else if (access->event) {
      invalidate_event_results (*access->event);
    }
  }
  void invalidate_prediction (prediction_type & prediction) {
    prediction.history = nullptr;
    prediction.previous = nullptr;
    prediction.result_function = nullptr;
    prediction.latest_access = nullptr;
    while (prediction.last_event) {
      invalidate_event_completely (*prediction.last_event);
      prediction.last_event = prediction.last_event->previous_from_same_prediction;
    }
  }
  void invalidate_prediction_after (prediction_type & prediction, extended_time time) {
    assert (time < prediction.valid_until);
    if (time <= prediction.made_at) {
      invalidate_prediction (*prediction);
    }
    else {
      prediction.valid_until = time;
      prediction.latest_access.reset (new access_info {prediction.valid_until, nullptr, &prediction});
      while (prediction.last_event && prediction.last_event->time >= time) {
        invalidate_event_completely (*prediction.last_event);
        prediction.last_event = prediction.last_event->previous_from_same_prediction;
      }
    }
  }
  void invalidate_event_completely (event_type & event) {
    assert (event.valid);
    event.valid = false;
    if (event.executed) {
      upcoming_issues.insert (EVENT_INCORRECTLY_EXECUTED, & event);
    }
  }
  void invalidate_event_results (event_type & event) {
    assert (event.executed);
    assert (event.access);
    event.access->event = nullptr;
    event.access = nullptr;
    upcoming_issues.insert (EVENT_INCORRECTLY_EXECUTED, & event);
  }
  void record_access (field_ID ID, access_pointer const & access) {
    auto & history = access_histories [ID];
    if (accessor->time == max_time) {
      history.current_prediction_accesses.push_back (access);
    }
    else {
      history.bounded_accesses.push (access);
    }
  }
  
  void do_event (event_type & event) {
    event_accessor accessor (this, event.time);
    event.function () (accessor);
    event.executed = true;
    event.access.reset (new access_info {event.time, & event, nullptr});
    for (auto accessed: accessor.accessed) {
      record_access (accessed, event.access);
    }
    event.modified = std:: move (accessor.modified);
    std:: remove_if (event.modified.begin (), event->modified.end (),
      [] (field_ID const & ID) {
      if ((*transfer_field_change_functions [ID.field_index]) (accessor, ID.entity)) {
        invalidate_accesses_since (event->time, ID);
        return false;
      }
      return true;
    });
  }
  void undo_event (event_reference event) {
    for (auto ID: event->modified) {
      (*undo_field_change_functions [ID.field_index]) (event->time, ID.entity);
      invalidate_accesses_since (event->time, ID, true);
    }
  }
  void make_prediction (extended_time when, predictor_index predictor, entity_ID entity) {
    prediction_type & prediction = predictions [next_serial_number++];
    prediction.made_at = when;
    predictor_accessor accessor (this, prediction);
    physics.run_predictor (predictor, accessor, entity);
    //that sets prediction.valid_until and prediction.prediction
    prediction.latest_access.reset (new access_info {prediction.valid_until, prediction.serial_number});
    for (auto accessed: accessor.accessed) {
      record_access (accessed, prediction->latest_access);
    }
    if (prediction.valid_until <max_time) {
      upcoming_issues.insert (MISSING_PREDICTION, predictor, prediction.valid_until);
    }
    if (prediction.prediction && next_event_time (prediction) <= prediction.valid_until) {
      upcoming_issues.insert (MISSING_PREDICTED_EVENT, prediction);
    }
  }
  void handle_issue (issue_type issue) {
    switch (issue.type) {
      case MISSING_PREDICTION: {
        make_prediction (issue.time, issue.predictor, issue .id);
      } break;
      case MISSING_PREDICTED_EVENT: {
        
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
