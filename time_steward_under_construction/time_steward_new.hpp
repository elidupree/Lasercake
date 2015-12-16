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
  
  
  
  template <typename field_identifier>
  struct field_history {
    typedef physics:: data_for <field_identifier> field_data;
    typedef std:: pair <access_pointer, field_data> change;
    
    std:: vector <change> changes;
    std:: vector <access_pointer> existence_changes;
    physics:: data_for_each_predictor_watching <field_identifier, prediction_history> prediction_histories;
    
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
    extended_time & valid_until () {return event.time;}
    time_type & result_time () {return event.time.base_time;}
    access_pointer latest_access;
    
    event_type event;
  };
  struct prediction_history {
    predictor_index predictor;
    entity_ID entity;
    intrusive_pointer <prediction_type> last;
  }
  struct event_type: public boost::intrusive_ref_counter <event_type> {
    extended_time time;
    event_function function;
    //the following 2 members could be compressed in memory a bit,
    //if that turns out to be valuable
    bool executed;
    std::vector <entity_ID> modified;
  };
  struct access_info: public boost::intrusive_ref_counter <access_info> {
    //only one pointer is non-null at a time, so we could compress this
    extended_time time;
    event_type* event;
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
  enum issue_type {
    MISSING_NEXT_PREDICTION,
    MISSING_PREDICTED_EVENT,
    EVENT_NOT_CORRECTLY_EXECUTED
  };
  struct issue {
    issue_type type;
    extended_time time;
    intrusive_pointer <event_type> event;
    prediction_history*prediction;
  };
  struct issue_collector {
    struct sort {bool operator< (issue const & alpha, issue const & beta) const {
      //reverse ordering, to do the earliest issues first
      if (alpha.time != beta.time) return alpha.time >beta.time;
      //for some reason, maybe we prefer doing some issues first
      return alpha.type >beta.type;
    }};
    std::priority_queue <issue, std::vector <issue>, sort> data;
    void insert (issue added) {data.push (added);}
  }
  
  
  
  physics:: data_for_each_field <field_history_map> field_histories;
  //should access histories be part of the field histories,
  //like the prediction histories are?
  //I'm keeping them separate, because I plan to make it so that
  //access histories are not always kept for every field, and instead,
  //can sometimes apply to a whole group of fields, to minimize the number
  //of accesses we have to store.
  std::unordered_map <field_ID, access_history> access_histories;
  //the only function of the fiat events container is to allow the client
  //to delete fiat events by time/distinguisher.
  //Some optimization could be done here.
  //It's low priority because there are usually fewer fiat events than other things.
  std::unordered_map <siphash_id, intrusive_pointer <event_type>> fiat_events;
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
    
    //note: if we are ever allowed to delete things in the middle of a
    //field history, without deleting everything after it, then it becomes
    //necessary to store ALL cases in which an event set the field state,
    //even if it wasn't a change at the processing-time* when the event
    //was first executed.
    //However, since we only delete an entire history tail at a time,
    //any redundant changes we stored would be deleted anyway,
    //so we don't have to store them.
    if (!exists_before &&!exists_after) {return false;}
    //here would be the place to optimize by returning false if the
    //DATA before and after were equal, if they are equality comparable
    
    erase_field_changes_since <field_identifier> (history, accessor.now ());
    if (exists_after) {
      history.changes.emplace_back (std:: move (accessor.entities.get <field_identifier> ()));
      if (!exists_before) {
        history.existence_changes.push_back (accessor.now ());
        for (auto const & prediction_history: history.prediction_histories) {
          upcoming_issues.insert {MISSING_NEXT_PREDICTION, accessor.now (), nullptr, & prediction_history};
        };
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
    if (there's still a change at time) {
      erase_field_changes_since <field_identifier> (history, time);
    }
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
    while (history.bounded_accesses.top ()->time >time ||
        //when we undo an event, we also want to remove all references
        //to that event, including old predictions that were valid until that event.
        (undoing_event && history.bounded_accesses.top ()->time == time)) {
      invalidate_access (time, history, history.bounded_accesses.top (), undoing_event);
      history.bounded_accesses.pop ();
    }
  }
  void invalidate_access (extended_time time, access_history & history, access_pointer const & access, bool undoing_event) {
    if (access->prediction) {
      auto & history =*access->prediction->history;
      bool needs_replacement = access->prediction.made_at <time;
      if (time <access->prediction->valid_until () ||
          //when we undo an event, we also want to remove all references
          //to that event, including old predictions that were valid until that event.
          (undoing_event && time == access->prediction->valid_until ())) {
        while (history.last != access->prediction) {
          assert (history.last->made_at >= time);
          invalidate_prediction (*history.last);
        }
        if (undoing_event) {
          //adding a change can partially invalidate a prediction,
          //but removing one can't.
          assert (time == access->prediction->valid_until);
          //for the "remove all references" thing, we have to destroy the
          //whole prediction. If the prediction stored a list of things
          //that it accessed, we could, instead, re-extend the prediction
          //into the future. But it doesn't (to save memory).
          invalidate_prediction (*access->prediction, time);
        }
        else {
          invalidate_prediction_after (*access->prediction, time);
        }
        if (history.last&& field_exists_after (physics:: field_for_predictor (history->predictor), history->entity, history.last->valid_until)) {
          upcoming_issues.insert {MISSING_NEXT_PREDICTION, history.last->valid_until, nullptr, & history};
        }
      }
      if (needs_replacement) {
        history.bounded_accesses.push (prediction.latest_access);
        //the above may invalidate the variable "access".
        //Therefore, we can't use access after this. Just be safe,
        return;
      }
    }
    else if (access->event) {
      invalidate_event_results (*access->event);
    }
  }
  void invalidate_prediction (prediction_type & prediction) {
    assert (prediction.history->last == & prediction);
    prediction.history->last = prediction.previous;
    prediction.history = nullptr;
    prediction.previous = nullptr;
    prediction.latest_access = nullptr;
    invalidate_event_completely (prediction.event);
  }
  void invalidate_prediction_after (prediction_type & prediction, extended_time time) {
    assert (time < prediction.valid_until ());
    if (time <= prediction.made_at) {
      invalidate_prediction (*prediction);
    }
    else {
      invalidate_event_completely (prediction.event);
      prediction.valid_until () = time;
      prediction.latest_access.reset (new access_info {prediction.valid_until (), nullptr, &prediction});
    }
  }
  void invalidate_event_completely (event_type & event) {
    assert (event.function);
    event.function = nullptr;
    if (event.executed) {
      invalidate_event_result (event);
    }
  }
  void invalidate_event_results (event_type & event) {
    assert (event.executed);
    assert (event.access);
    event.access->event = nullptr;
    event.access = nullptr;
    upcoming_issues.insert (EVENT_NOT_CORRECTLY_EXECUTED, & event);
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
  
  //TODO: when we invalidate event results,
  //combine do_event and undo_event so that any fields that are changed
  //the same way as they were changed last time don't need to have their
  //accesses invalidated.
  void do_event (event_type & event) {
    event_accessor accessor (this, event.time);
    event.function () (accessor);
    event.executed = true;
    event.access.reset (new access_info {event.time, & event, nullptr});
    for (auto accessed: accessor.accessed) {
      record_access (accessed, event.access);
    }
    event.modified = std:: move (accessor.modified);
    std:: remove_if (event.modified.begin (), event.modified.end (),
      [] (field_ID const & ID) {
      if ((*transfer_field_change_functions [ID.field_index]) (accessor, ID.entity)) {
        invalidate_accesses_since (event.time, ID);
        return false;
      }
      return true;
    });
    //we could spend CPU to save memory, by doing
    //event.modified.shrink_to_fit ();
    //the might be a way to end up with an exact-sized container
    //without spending extra CPU, hmm
  }
  void undo_event (event & event) {
    for (auto ID: event.modified) {
      (*undo_field_change_functions [ID.field_index]) (event.time, ID.entity);
      invalidate_accesses_since (event.time, ID, true);
    }
    event.modified.clear ();
    event.modified.shrink_to_fit ();
  }
  
  void make_prediction (extended_time when, intrusive_pointer <prediction_type> const & previous) {
    intrusive_pointer <prediction_type> prediction_pointer (new prediction_type);
    prediction_type & prediction =*prediction_pointer;
    prediction.made_at = when;
    prediction.previous = previous;
    prediction.history = previous->history;
    
    predictor_accessor accessor (this, prediction);
    physics.run_predictor (predictor, accessor, previous->history->entity);
    //that sets prediction.event.time and prediction.event.function
    
    prediction.latest_access.reset (new access_info {prediction.valid_until (), nullptr, prediction_pointer});
    for (auto accessed: accessor.accessed) {
      record_access (accessed, prediction->latest_access);
    }
    
    if (prediction.valid_until () <max_time && field_exists_after (physics:: field_for_predictor (prediction.history->predictor), prediction.history->entity, when)) {
      upcoming_issues.insert {MISSING_NEXT_PREDICTION, prediction.valid_until, nullptr, prediction.history};
    }
    prediction_may_predict_event (prediction);
  }
  void prediction_may_predict_event (prediction_type & prediction) {
    extended_time event_time = next_event_time (prediction);
    if (prediction.result_function && event_time >prediction.made_at && event_time <= prediction.valid_until) {
      upcoming_issues.insert {MISSING_PREDICTED_EVENT, event_time, nullptr, & prediction};
    }
  }
  
  extended_time prediction_history_next_attention_time (prediction_history const & history) {
    extended_time search_from;
    if (history.last) {
      if (history.last->result_function) {
        extended_time event_time = next_event_time (*history.last);
        if (event_time >prediction.made_at && event_time <= prediction.valid_until) {
          return event_time;
        }
      }
      search_from = history.last->valid_until;
    }
    else {
      search_from = min_extended_time;
    }
    auto transition = next_existence_transition (field_for_predictor (history.predictor), history.entity, search_from);
    if (transition.started_existing) return transition.time;
    return search_from;
  }
  
  siphash_id predicted_event_time_ID (prediction_history const & history, time_type base_time, uint64_t iteration) {
    return siphash_id::combining (history.predictor, history.entity, base_time, iteration);
  }
  siphash_id fiat_event_time_ID (time_type time, uint64_t distinguisher) {
    //TODO: make this part of time_traits
    return siphash_id::combining (time, distinguisher);
  }
  extended_time next_event_time (prediction_type const & prediction) {
    prediction_history const & history =*prediction.history;
    uint64_t iteration;
    if (prediction.last_event) {
      iteration = prediction.last_event->time.iteration + 1;
    }
    else if (prediction.result_time == prediction.made_at.base_time) {
      siphash_id candidate_ID = predicted_event_time_ID (history, prediction.result_time, prediction.iteration);
      if (candidate_ID >prediction.made_at.ID) {
        return {prediction.made_at.base_time, prediction.made_at.iteration, candidate_ID};
      }
      iteration = prediction.made_at.iteration + 1;
    }
    else {
      iteration = 0;
    }
    return {prediction.result_time, iteration, predicted_event_time_ID (history, prediction.result_time, iteration)};
  }
  
  void handle_issue (issue_type issue) {
    switch (issue.type) {
      case MISSING_NEXT_PREDICTION: {
        if (prediction_history_next_attention_time (*issue.predictor) == issue.time) {
          make_prediction (issue.time,*issue.predictor);
        }
      } break;
      case MISSING_PREDICTED_EVENT: {
        if (prediction_history_next_attention_time (*issue.predictor) == issue.time) {
          make_predicted_event (issue.time,*issue.predictor);
        }
      } break;
      case EVENT_NOT_CORRECTLY_EXECUTED: {
        event_type & event =*issue.event;
        if (event.executed &&!event.access) {
          undo_event (event);
        }
        if (event.valid) {
          do_event (event);
        }
      } break;
    }
  }
  
public:
  void insert_fiat_event (time_type time, uint64_t distinguisher, event_function function) {
    extended_time time = {time, 1ull << 63, fiat_event_time_ID (time, distinguisher)};
    event_type & event = fiat_events [time .ID];
    event.time = time;
    event.function = function;
    upcoming_issues.insert {EVENT_NOT_CORRECTLY_EXECUTED, time, event_pointer, nullptr};
  }
  void erase_fiat_event (time_type time, uint64_t distinguisher) {
    auto iterator = fiat_events.find (fiat_event_time_ID (time, distinguisher));
    if (iterator != fiat_events.end ()) {
      if (iterator->second.executed) {
        undo_event (iterator->second);
      }
      fiat_events.erase (iterator);
    }
  }
};

}//End namespace implementation

using implementation:: time_steward;
using implementation:: physics_list;

}//end namespace time_steward_system

#endif
