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

struct empty_list {};
template <typename head_type, class tail_type> struct nonempty_list {typedef head_type head; typedef tail_type tail;};
template <typename... Input> struct list_flatten_1;
template <> struct list_flatten_1 <> {typedef empty_list type;};
template <typename list_head, typename list_tail, typename... tail> struct list_flatten_1 <nonempty_list <list_head, list_tail>, tail...> {typedef list_flatten_1 <list_head, list_tail, tail...>::type type;};
template <typename head, typename... tail> struct list_flatten_1 <head, tail...> {typedef nonempty_list <head, list_flatten_1 <tail...>::type type;};
template <typename... input> using flatten_list = typename list_flatten_1 <input...>::type;

template <class list, class filter> struct list_filter_1 {typedef list_filter_2 <list, filter>::type type;};
template <class filter> struct list_filter_1 <empty_list, filter> {typedef empty_list type;};
template <class list, class filter, bool head_passes = filter::passes <list::head>> struct list_filter_2;
template <class list, class filter> struct list_filter_2 <list, filter, true> {struct type {typedef list::head head; typedef list_filter_1 <list::tail, filter> ::type tail;};};
template <class list, class filter> struct list_filter_2 <list, filter, false> {typedef list_filter_1 <list::tail, filter>::type type;};
template <class list, class filter> using filter_list = typename list_filter_1 <list, filter>::type;

template <typename ID, typename list, bool equal = std::is_same <ID, list:: head:: identifying_type>> struct field_selector;
template <typename ID, typename list> struct field_selector <ID, list, true> {typedef list::head type;};
template <typename ID, typename list> struct field_selector <ID, list, false> {typedef field_selector <ID, list::tail>::type;};
template <typename ID, typename list> using select_field <ID, list> = typename field_selector <ID, list>::type;

struct field_base {};
template <typename field_identifier, typename field_data>
struct field: public field_base {
  typedef field_identifier identifying_type;
  typedef field_data data_type;
};

struct predictor_base {};
template <typename field_identifier, template <typename Accessor> void (Accessor, entity_ID) function>
struct predictor: public predictor_base {
  typedef field_identifier field;
  template <typename Accessor>
  static inline void call (Accessor accessor, entity_ID ID) {
    function <Accessor> (accessor, ID);
  }
};

template <typename base>
struct base_filter {template <typename input> static constexpr bool passes () {return std::is_base_of <base, input>::value;}};

struct default_time_traits {
  typedef int64_t type;
  static const type never = std::numeric_limits <type>::min ();
  static const type min = never + 1;
  static const type max = std::numeric_limits <type>::max ();
};

template <class list, class time_traits_type = default_time_traits>
struct physics {
  typedef filter_list <list, base_filter <field_base>> fields;
  typedef filter_list <list, base_filter <predictor_base>> predictors;
  typedef time_traits_type time_traits;
  struct extended_time {
    time_traits::type base_time;
    uint64_t iteration;
    siphash_id ID;
    
    static const extended_time max {time_traits::max, uint64_t (-1), siphash_id::greatest ()};
    static const extended_time min {time_traits::min, 0, siphash_id::least ()};
    static const extended_time never {time_traits::never, 0, siphash_id::null ()};
    
    explicit operator bool () const {return bool (ID);}
    bool operator< (extended_time const & other) const {
      if (base_time != other.base_time) return base_time <other.base_time;
      if (iteration != other.iteration) return iteration <other.iteration;
      return ID <other.ID;
    }
    bool operator<=(extended_time const & other) const {
      if (base_time != other.base_time) return base_time <other.base_time;
      if (iteration != other.iteration) return iteration <other.iteration;
      return ID <=other.ID;
    }
    bool operator> (extended_time const & other) const {
      if (base_time != other.base_time) return base_time >other.base_time;
      if (iteration != other.iteration) return iteration >other.iteration;
      return ID >other.ID;
    }
    bool operator>=(extended_time const & other) const {
      if (base_time != other.base_time) return base_time >other.base_time;
      if (iteration != other.iteration) return iteration >other.iteration;
      return ID >=other.ID;
    }
    bool operator== (extended_time const & other) const {
      return other.base_time == base_time && other.iteration == iteration && other.ID == ID;
    }
    bool operator== (extended_time const & other) const {
      return other.base_time != base_time || other.iteration != iteration || other.ID != ID;
    }
  };
};

template <template <typename> class data, class fields>
struct data_for_each_field {
  typedef fields::head::identifying_type head_ID;
  data <head_ID> head;
  data_for_each_field <data, fields::tail> tail;
  template <typename ID> inline std::enable_if_t <std::is_same <ID, head_ID>::value, data <ID>> & get () {return head;}
  template <typename ID> inline std::enable_if_t <!std::is_same <ID, head_ID>::value, data <ID>> & get () {return tail.get <ID> ();}
}
template <template <typename> class data>
struct data_for_each_field <data, empty_list> {};

template <template <typename> class function, class list, typename... Arguments>
inline void call_for_each (Arguments const &... arguments);
template <template <typename> class function, class head, class tail, typename... Arguments>
inline void call_for_each <function, nonempty_list <head, tail>, Arguments> (Arguments &&... arguments) {
  function<head>::apply (arguments...);
  call_for_each <function, tail, Arguments> (arguments...);
}
template <template <typename> class function, typename... Arguments>
inline void call_for_each <function, empty_list, Arguments> (Arguments const &... arguments) {}

template <typename ID, typename physics> using data_for <ID, physics> = typename select_field <ID, physics::fields>::data_type;

template <typename Alpha, typename Bravo>
auto verifiably_equal (Alpha const & alpha, Bravo const & bravo)
->decltype (alpha == bravo) {return alpha == bravo;}
bool verifiably_equal (...) {return false;}

template <typename Alpha, typename Bravo>
auto verifiably_unequal (Alpha const & alpha, Bravo const & bravo)
->decltype (alpha != bravo) {return alpha != bravo;}
bool verifiably_unequal (...) {return false;}

struct field_ID {
  entity_ID entity;
  uint32_t field_index;
  bool operator== (field_ID const & other) const {return entity == other.entity && field_index == other.field_index;}
};
}//End namespace implementation
}//end namespace time_steward_system
namespace std {
template <> struct hash <time_steward_system::implementation::field_ID> {public: size_t operator () (time_steward_system::implementation::field_ID const & ID) const {return std::hash <siphash_id> (ID.entity) ^ ID.field_index;}};
}
namespace time_steward_system {
namespace implementation {


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
  MINIMAL,  
  CONSTANT_FACTOR,
  HISTORY_LENGTH_FACTOR,
  ENTIRE_STEWARD_FACTOR
};
using intrusive_pointer = boost::intrusive_ptr
template <typename physics, auditing_level_type auditing_level>
class time_steward {
  
  typedef std:: function <void (event_accessor &)> event_function;
  
  class event_accessor {
  private:  
    friend class time_steward;
    
    struct field_info_base {
      bool accessed_old_state;
      bool possibly_modified;
    };
    template <typename field_identifier>
    struct field_info: public field_info_base {
      typedef physics:: data_for <field_identifier> field_data;
      boost::optional <field_data > data;
      field_info (field_data const & data, accessed, modified):
        data (data), accessed_old_state (accessed), possibly_modified (modified) {}
    };

    time_steward const*steward_;
    extended_time time_;
    siphash_random_generator RNG_;
    
    mutable field_map <field_info_base, field_info> entities_;
    
    event_accessor (time_steward const*steward, extended_time time):
      steward_ (steward), time_(time), RNG_(time.ID) {}
    
    template <typename field_identifier>
    inline field_info <field_identifier> & get_field_info (entity_ID ID, bool accessed, bool modified) const {
      auto result = entities_.get_with_default <field_identifier> (ID, [ID] () {
        auto probe = steward_->probe_field <field_identifier, false> (ID, time_);
        return field_info <field_identifier> (probe.value, accessed, modified);
      });
      if (result.second && modified) {result.first.possibly_modified = true;}
    }
    template <typename field_identifier>
    inline physics:: data_for <field_identifier> & set_implementation (entity_ID ID, physics: data_for <field_identifier> const & new_data) {
      auto & map = entities_.get <field_identifier> ();
      auto & iterator = map.find (ID);
      if (iterator == map.end ()) {
        auto result = map.emplace (ID, new_data, false, true);
        assert (result.second);
        iterator = result.first;
      }
      return iterator->second.data;
    }

  public:
    template <typename field_identifier>
    inline bool exists (entity_ID ID) const {
      //theoretically, there's a distinction between accessing data
      //and merely checking existence. However, I doubt that will be useful.
      return bool (get_field_info (ID, true, false).data);
    }
    
    template <typename field_identifier>
    inline physics:: data_for <field_identifier> const & get (entity_ID ID) const {
      auto & info = get_field_info (ID, true, false);
      caller_correct_if (info.data, "you can't get() a field that doesn't exist");
      return *info.data;
    }
    template <typename field_identifier>
    inline physics:: data_for <field_identifier> & get_mutable (entity_ID ID) {
      auto & info = get_field_info (ID, true, true);
      caller_correct_if (info.data, "you can't get_mutable() a field that doesn't exist");
      return *info.data;
    }
    template <typename field_identifier, typename... Arguments>
    inline physics:: data_for <field_identifier> & set (entity_ID ID, Arguments... arguments) {
      auto & info = get_field_info (ID, false, true);
      info.emplace (std::forward <Arguments> (arguments)...);
      return *info.data;
    }
    
    uint64_t random_bits (uint32_t bits) {
      return RNG_.random_bits (bits);
    }
    siphash_id random_id () {
      return RNG_.random_id ();
    }
    time_type now () const {
      return time_.base_time;
    }
    event_accessor (event_accessor const &) = delete;
    event_accessor (event_accessor &&) = delete;
    event_accessor & operator= (event_accessor const &) = delete;
    event_accessor & operator= (event_accessor &&) = delete;
  };
  class predictor_accessor {
  private:
    friend class time_steward;
    
    time_steward const*steward_;
    extended_time time_;
    prediction_type*prediction_;
    
    std::unordered_set <field_ID> fields_accessed;
    
    predictor_accessor (time_steward const*steward, extended_time time, prediction_type*prediction):
      steward_(steward), time_(time), prediction_(prediction) {
      prediction_->valid_until () = max_extended_time;
    }
    
    inline void not_valid_after (extended_time time) {
      if (time < prediction_->valid_until ()) {
        prediction_->valid_until () = time;
        prediction_->event.function = nullptr;
      }
    }
  public:
    template <typename field_identifier>
    inline physics:: data_for <field_identifier> const & get (entity_ID ID) {
      //TODO: the unordered_map is only for uniquing. As an optimization,
      //we should be able to use a boost::small_vector and a borrowed_bitset.
      fields_accessed.emplace (ID, physics:: field_index <field_identifier> ());
      auto result = steward_->probe_field <field_identifier, true> (ID, time_);
      not_valid_after (result.end_time);
      caller_correct_if (result.value, "you can't get() a field that doesn't exist");
      return *result.value;
    }
    template <typename field_identifier>
    inline bool exists (entity_ID ID) {
      //TODO: the unordered_map is only for uniquing. As an optimization,
      //we should be able to use a boost::small_vector and a borrowed_bitset.
      fields_accessed.emplace (ID, physics:: field_index <field_identifier> ());
      auto result = steward_->probe_field <field_identifier, true> (ID, time_);
      not_valid_after (result.end_time);
      return bool (result.value);
    }
    inline void predict (time_type predicted_time, event_function predicted_function) {
      if (predicted_time < time_.base_time) { return; }
      if (predicted_time > prediction_->valid_until ().base_time) {return;}
      extended_time event_time;
      event_time.base_time = predicted_time;
      if (predicted_time == time_.base_time) {
        siphash_id candidate_ID = predicted_event_time_ID (
          prediction_->history->predictor, prediction_->history->entity,
          predicted_time, time_.iteration);
        if (candidate_ID >time_.ID) {
          event_time.iteration = time_.iteration;
          event_time .ID = candidate_ID;
        }
        else {
          event_time.iteration = time_.iteration + 1;
        }
      }
      else {
        event_time.iteration = 0;
      }
      if (!event_time.ID) {
        event_time .id =predicted_event_time_ID (
          prediction_->history->predictor, prediction_->history->entity,
          predicted_time, event_time.iteration);
      }
      if (event_time >prediction_->valid_until ()) {return;}
      prediction_->event.function = predicted_function;
      prediction_->event.time = event_time;
    }
    
    predictor_accessor (predictor_accessor const &) = delete;
    predictor_accessor (predictor_accessor &&) = delete;
    predictor_accessor & operator= (predictor_accessor const &) = delete;
    predictor_accessor & operator= (predictor_accessor &&) = delete;
  };
  class outer_accessor {
  private:
    time_steward const*steward_;
    extended_time time_;
    
    outer_accessor (time_steward const*steward, extended_time time):
      steward_(steward), time_(time) {}
  public:
    template <typename field_identifier>
    inline physics:: data_for <field_identifier> const & get (entity_ID ID) {
      auto result = steward_->probe_field <field_identifier, false> (ID, time_);
      caller_correct_if (result.value, "you can't get() a field that doesn't exist");
      return *result.value;
    }
    template <typename field_identifier>
    inline bool exists (entity_ID ID) {
      auto result = steward_->probe_field <field_identifier, false> (ID, time_);
      return bool (result.value);
    }
    time_type now () const {
      return time_.base_time;
    }
    //you're still not supposed to store these things, because they might become invalid,
    //but unlike the other accessor types, it's not inherently unsafe to copy them,
    //so we don't delete the copy constructors and assignment operators
    //like we do with other accessor types.
  };
  
  class predictor_accessor_pointer {
  private:
    predictor_accessor*data;
  public:
    predictor_accessor*operator->() const {return data;}
    typedef event_accessor*for_event;
  };
  
  struct field_history_base {
  
  };
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
  typedef intrusive_pointer <access_info> access_pointer;
  struct prediction_history;
  struct prediction_type: public boost::intrusive_ref_counter <prediction_type> {
    //null if this prediction is invalid
    prediction_history_type*history;
    //this could presumably be a regular pointer,
    //but for now, I'm just going to be extra memory-safe.
    intrusive_pointer <prediction_type> previous;
    extended_time made_at;
    extended_time & valid_until () {return event.time;}
    time_type & result_time () {return event.time.base_time;}
    access_pointer latest_access;
    
    event_type event;
  };
  struct prediction_history_type {
    predictor_index predictor;
    entity_ID entity;
    access_pointer issue;
    intrusive_pointer <prediction_type> last;
  }
  struct event_type: public boost::intrusive_ref_counter <event_type> {
    extended_time time;
    event_function function;
    bool executed_correctly;
    access_pointer access;
    bool executed () const {return bool (access);}
    std::vector <entity_ID> modified;
  };
  struct access_info: public boost::intrusive_ref_counter <access_info> {
    //only one pointer is non-null at a time, so we could compress this.
    //Perhaps a boost::variant would be appropriate?
    extended_time time;
    event_type* event;
    intrusive_pointer <prediction_type> prediction;
    prediction_history_type*prediction_history;
    access_info (extended_time time, event_type*event):
      time (time), event (event), prediction (nullptr), prediction_history (nullptr) {}
    access_info (extended_time time, intrusive_pointer <prediction_type> const & prediction):
      time (time), event (nullptr), prediction (prediction), prediction_history (nullptr) {}
    access_info (extended_time time, prediction_history_type*prediction_history):
      time (time), event (nullptr), prediction (nullptr), prediction_history (prediction_history) {}
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
  struct issue_old {
    issue_type type;
    extended_time time;
    intrusive_pointer <event_type> event;
    prediction_history*prediction;
  };
  typedef access_pointer issue;
  struct issue_collector {
    struct sort {bool operator< (issue const & alpha, issue const & beta) const {
      //reverse ordering, to do the earliest issues first
      if (alpha.time != beta.time) return alpha.time >beta.time;
      //for some reason, maybe we prefer doing some issues first
      return alpha.type >beta.type;
    }};
    std::priority_queue <issue, std::vector <issue>, sort> data;
    void insert (issue const & added) {data.push (added);}
    
  }
  
  
  
  physics:: data_for_each_field <field_history_map> field_histories;
  //should access histories be part of the field histories,
  //like the prediction histories are?
  //I'm keeping them separate, because I plan to make it so that
  //access histories are not always kept for every field, and instead,
  //can sometimes apply to a whole group of fields, to minimize the number
  //of accesses we have to store.
  std::unordered_map <field_ID, access_history> access_histories;
  //the keys are redundant with the ID in the event time.
  //Some optimization could be done here.
  //It's low priority because there are usually fewer fiat events than other things.
  std::unordered_map <siphash_id, event_type> fiat_events;
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
    if (exists_before && exists_after && verifiably_equal
      (history.data_before (accessor.now ()),
       accessor.entities.get <field_identifier> (ID))) {return false;}
    
    erase_field_changes_since <field_identifier> (history, accessor.now ());
    if (exists_after) {
      history.changes.emplace_back (std:: move (accessor.entities.get <field_identifier> (ID)));
      if (!exists_before) {
        history.existence_changes.push_back (accessor.now ());
        for (auto const & prediction_history: history.prediction_histories) {
          note_prediction_history_issue (prediction_history, accessor.now ());
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
        note_next_prediction_history_issue (history); 
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
      prediction.latest_access.reset (new access_info (time, prediction));
    }
  }
  void invalidate_event_completely (event_type & event) {
    assert (event.function);
    event.function = nullptr;
    if (event.executed ()) {
      invalidate_event_result (event);
    }
  }
  void invalidate_event_results (event_type & event) {
    assert (event.access);
    if (event.executed_correctly) {
      event.executed_correctly = false;
      upcoming_issues.insert (event.access);
    }
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
    event.access.reset (new access_info (event.time, & event));
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
    event.executed_correctly = true;
  }
  void undo_event (event & event) {
    for (auto ID: event.modified) {
      (*undo_field_change_functions [ID.field_index]) (event.access->time, ID.entity);
      invalidate_accesses_since (event.access->time, ID, true);
    }
    event.modified.clear ();
    event.modified.shrink_to_fit ();
    event.access->event = nullptr;
    event.access = nullptr;
  }
  
  void note_prediction_history_issue (prediction_history_type & history, extended_time time) {
    if (time == extended_time::max) return;
    if (!history.issue || history.issue.time >time) {
      if (history.issue) history.issue->prediction_history = nullptr;
      history.issue.reset (new access_info (time, & history);
      upcoming_issues.insert (history.issue);
    }
  }
  void next_prediction_history_issue_time (prediction_history_type const & history) {
    extended_time search_from;
    if (history.last) {
      if (history.last->event.function && !history.last->event.executed ()) {
        return history.last->event.time;
      }
      search_from = history.last->valid_until ();
    }
    else {
      search_from = min_extended_time;
    }
    auto transition = next_existence_transition (field_for_predictor (history.predictor), history.entity, search_from);
    if (transition.started_existing) {
      return transition.time;
    }
    return search_from;
  }
  void note_next_prediction_history_issue (prediction_history_type & history) {
    note_prediction_history_issue (next_prediction_history_issue_time (history));
  }  
  
  void make_prediction (extended_time when, intrusive_pointer <prediction_type> const & previous) {
    intrusive_pointer <prediction_type> prediction_pointer (new prediction_type);
    prediction_type & prediction =*prediction_pointer;
    prediction.made_at = when;
    prediction.previous = previous;
    prediction.history = previous->history;
    prediction.history->last = prediction_pointer;
    
    predictor_accessor accessor (this, prediction);
    physics.run_predictor (predictor, accessor, previous->history->entity);
    //that sets prediction.event.time and prediction.event.function
    
    prediction.latest_access.reset (new access_info (prediction.valid_until (), prediction_pointer));
    for (auto accessed: accessor.accessed) {
      record_access (accessed, prediction->latest_access);
    }
    
    note_next_prediction_history_issue (prediction.history);
  }
  
  static siphash_id predicted_event_time_ID (prediction_history const & history, time_type base_time, uint64_t iteration) {
    return siphash_id::combining (history.predictor, history.entity, base_time, iteration);
  }
  static siphash_id fiat_event_time_ID (time_type time, uint64_t distinguisher) {
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
  
  bool issue_valid (access_pointer const & issue_pointer) {
    auto & issue =*issue_pointer;
    if (issue.event) {
      return issue.event->access == & issue;
    }
    if (issue.prediction_history) {
      return issue.prediction_history->issue == & issue;
    }
    return false;
  }
  void handle_issue (access_pointer const & issue_pointer) {
    auto & issue =*issue_pointer;
    if (issue.event) {
      event_type & event =*issue.event;
      assert (!event. executed_correctly);
      if (event.access) {
        undo_event (event);
      }
      if (event.function) {
        do_event (event);
      }
    }
    if (issue.prediction_history) {
      prediction_history_type & history =*issue.prediction_history;
      if (history.last && history.last->event.function && ! history.last->event.executed ()) {
        do_event (history.last->event);
      }
      history.issue = nullptr;
      make_prediction (issue.time, history.last);
    }
  }
  
  //some validation functions to make sure the invariants are maintained.
  //It's unfortunate that the words "validate", for these functions,
  //and "invalidate", for the others, have completely unrelated meanings.
  void validate_prediction_history_constant (prediction_history_type const & history) {
    //each history has a valid issue if and only if it should
    extended_time time = next_prediction_history_issue_time (history);
    if (time == extended_time::max) assert (!history.issue);
    else {
      assert (history.issue);
      assert (history.issue->prediction_history == & history);
      assert (history.issue->time == time);
      validate_access (*history.issue);
    }
    if (history.last) {
      assert (time >= history.last->valid_until ());
    }
  }
  void validate_access (access_info const & access) {
    //an access only has one pointer, and except for predictors, it must
    //still be current
    if (access.event) {
      assert (!access.prediction);
      assert (!access.prediction_history);
      assert (access.event->access == & access);
      //note that the event is NOT required to have the same time as the access
    }
    if (access.prediction_history) {
      assert (!access.prediction);
      assert (access.prediction_history->issue == & access);
    }
  }
  void validate_prediction (prediction_type const & prediction) {
    //predictions must come in order
    if (prediction.previous) {
      assert (prediction.previous->valid_until () <= prediction.made_at);
    }
    //if valid, predictions must cover any time, and have a valid access
    if (prediction.history) {
      assert (prediction.valid_until () >prediction.made_at);
      assert (prediction.latest_access);
      assert (prediction.latest_access->time == prediction.valid_until ());
      validate_event (prediction.event);
    }
  }
  void validate_event (event_type const & event) {
    if (event.executed_correctly) {
      assert (event.executed ());
      assert (event.access->time == event.time);
    }
    if (event.modified.size () >0) assert (event.executed ());
    if (event.access) {
      assert (event.access->event == & event);
      validate_access (*event.access);
    }
  }
  
  void validate_prediction_history_linear (prediction_history_type const & history) {
    validate_prediction_history_constant (history);
    for (prediction_type*prediction = history.last; prediction; prediction = prediction->previous) {
      validate_prediction (*prediction);
    }
  }
  
  template <class field>
  void validate_field_history_linear () {
    auto const & history = field_histories.get <field::identifying_type> ();
    for (auto prediction_history: history.prediction_histories) {
      validate_prediction_history_linear (prediction_history);
    }
    assert (std::is_sorted (history.changes.begin (), history.changes.end (), field_history_type::change_compare ());
    assert (std::is_sorted (history.existence_changes.begin (), history.existence_changes.end (), access_pointer_time_compare ());
    //never change to something without existing.
    //TODO: we can remove a log factor here by iterating the 2 arrays in parallel.
    //However, writing the code for the parallel iteration here would be
    //a very bad trade-off between performance and code complexity.
    //TODO: find or write a good template function for this parallel iteration.
    for (auto const & change: history.changes) {
      assert (exists_after (history.existence_changes, change.first->time));
    }
  }
  template <class field> struct validate_field_history_wrapper {
    void apply (time_steward*steward) {steward->validate_field_history_linear <field> ();}
  };
  
  void validate_everything () {
    for (auto & thing: fiat_events) validate_event (thing.second);
    call_for_each <validate_field_history_wrapper, physics::fields> (this);
    
  }
  
  //at the first field change in one steward that is not in the other,
  //stopping at the point where either time steward has an
  //outstanding issue that it knows about. 
  auto find_first_inconsistencies_with (time_steward const & other) const {
    extended_time best_time = extended_time::max;
    std::unordered_map <field_ID> fields_in_either;
    struct inconsistency {extended_time time; field_ID ID;};
    std::vector <inconsistency> inconsistencies;
    for (auto const & pair: fields) {fields_in_either.insert (pair.first);}
    for (auto const & pair: other.fields) {fields_in_either.insert (pair.first);}
    for (auto const & ID: fields_in_either) {
      auto my_iterator = fields.find (ID);
      auto other_iterator = other.fields.find (ID);
      extended_time first_fail = extended_time::max;
      if (my_iterator == fields.end ()) {
        first_fail = other_iterator->second.changes [0]->time;
      }
      else if (other_iterator == fields.end ()) {
        first_fail = my_iterator->second.changes [0]->time;
      }
      else {
        auto const & my_changes = my_iterator->second.changes;
        auto const & other_changes = other_iterator->second.changes
        for (size_t index = 0;;++index) {
          if (index == my_changes.size ()) {
            if (index != other_changes.size ()) {
              first_fail = other_changes [index]->time;
            }
            break;
          }
          else if (index == other_changes.size ()) {
            first_fail = my_changes [index]->time;
          }
          else {
            extended_time my_time = my_changes [index]->time;
            extended_time other_time = other_changes [index]->time;
            if (my_time != other_time) {
              first_fail = std::min (my_time, other_time);
              break;
            }
            //TODO: comparing the data
          }
        }
      }
      if (first_fail >= earliest_issue_time () || first_fail >= other.earliest_issue_time ()) {
        continue;
      }
      if (first_fail <best_time) {
        best_time = first_fail;
        inconsistencies.clear ();
      }
      if (first_fail == best_time) {
        inconsistencies.emplace_back {first_fail, ID};
      }
    }
    return inconsistencies;
    /*struct record {
      field_ID ID;
      extended_time time;
      bool operator< (record const & other) const {return time >other.time;}
    };
    std::priority_queue <record> records;
    std::unordered_map <field_ID> fields_found;
    call_for_each <add_starting_records, physics::fields> (records, fields_found, *this);
    call_for_each <add_starting_records, physics::fields> (records, fields_found, other);
    extended_time stop_after = extended_time::max;
    while (!records.empty ()) {
      auto const & top = records.top ();
      if (top.time >= earliest_issue_time () || top.time >= other.earliest_issue_time () || top.time >stop_after) {break;}
      
      records.pop ();
    }*/
  }
public:
  void insert_fiat_event (time_type time, uint64_t distinguisher, event_function function) {
    extended_time time = {time, 1ull << 63, fiat_event_time_ID (time, distinguisher)};
    event_type & event = fiat_events [time .ID];
    event.time = time;
    event.function = function;
    //special case
    event.access.reset (new access_info (event.time, & event));
    upcoming_issues.insert (event.access);
  }
  void erase_fiat_event (time_type time, uint64_t distinguisher) {
    auto iterator = fiat_events.find (fiat_event_time_ID (time, distinguisher));
    if (iterator != fiat_events.end ()) {
      //undo the event right away so that we can go ahead and delete it
      //without leaving around invalid references
      if (iterator->second.access) {
        undo_event (iterator->second);
      }
      fiat_events.erase (iterator);
    }
  }
  
  time_type first_possibly_incorrect_time () const {
    if (upcoming_issues.data.empty () ) {
      return max_time; 
    }
    return upcoming_issues.data.top ()->time;
  }
  void update_once () {
    if (!upcoming_issues.data.empty () ) {
      if (issue_valid (upcoming_issues.data.top ())) {
        handle_issue (upcoming_issues.data.top ());
      }
      upcoming_issues.data.pop ();
    }
  }
  void update_through (time_type time) {
    while (!upcoming_issues.data.empty () && upcoming_issues.data.top ()->time.base_time <= time) {
      if (issue_valid (upcoming_issues.data.top ())) {
        handle_issue (upcoming_issues.data.top ());
      }
      upcoming_issues.data.pop ();
    }
  }
};

}//End namespace implementation

using implementation:: time_steward;
using physics_list = implementation::flatten_list;

}//end namespace time_steward_system

#endif
