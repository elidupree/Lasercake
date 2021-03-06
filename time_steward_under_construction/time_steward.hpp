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



#include <array>
#include <set>
#include <unordered_set>
#include <map>
#include <unordered_map>
#include <inttypes.h>
#include <assert.h>
#include <limits>
#include <iostream>
#include <boost/next_prior.hpp>
#include <boost/optional.hpp>
#include <memory>
#include "siphash_id.hpp"
#include "time.hpp"
#include "bidirectional_interval_map.hpp"

/*
 
 I expect that this explanatory comment can be much improved.

 A time_steward sits outside the flow of time, looking down upon it.
 It wields impossible power over a universe of Entities.
 Its power comes in the form of FiatEvents, which are an
 event at a point in time imposed upon the world, such as, in a
 platform game, the player character jumping.

 There are also ConsequentialEvents, such as when that
 jumping player character smacks their head into a ceiling and
 thus stops moving upwards.

 In between those two Events, there is a state of
 continuous change.  It has no effect other than helping
 the (timesteward??) determine when the next ConsequentialEvent
 is, and letting the client determine a state of the world
 at a time in between events.



 Nothing else changes.

 


in a rocket
 game, the player character turning on their engines.

 
 
 A single time_steward manages one universe of Entities and their changes over time,
   1) such that the changes are discrete changes and occur at specific instants,
   2) such that, for a given time and Entity, the time and nature of its upcoming changes can be determined solely
        from the states of a small finite number of other entities,
   3) for the purpose of making a deterministic simulation with efficient access and mutation of the world state at arbitrary times.
 
 (3) allows various things:
   it can be used to smooth over networked games' lag (continue simulating locally assuming no network input,
     then go back and add the network input at the correct time as it arrives, and recompute only what is actually affected by that input).
   it can be used to smooth over momentary complicated situations (continuously simulate several seconds ahead,
     and add the user's input as it comes, similar to the above), and allows for parallelism (do multiple changes and overwrite any that happen to conflict).
   it allows us to give the user an interface into the world where ze can look back in time to any moment in the history,
     and perhaps try changing something, creating a new branch.
     In this manner, we can create a sandbox game with zero risk from momentary mistakes, since the user can scroll back in time whenever something goes wrong.
   The time_steward also has the advantage that neither Entities nor durations of time consume computer resources by themselves:
     only the Events need processing. Hence, any Entity that is seeing little action automatically yields processor time
     to other parts of the simulation - and if all Entities are fairly inactive, the time_steward can quickly pass a very large amount of time
     (as opposed to, for instance, having simulation steps that each require a minimal update).
     
 The rest of this comment essentially describes the technical meanings of (1) and (2) - 
   the restrictions we make on the universe in order to make these features possible.
 In this comment, we will discuss two types of things: Data, which exists in the program in a literal sense;
   and Facts, in the sense of mathematical facts, which are true but are not necessarily represented in the same way they are stated.
 The existence of "Entities" and "Events" is a mere Fact; when discussing Data, we will more specifically refer to EntityData, EntityIDs, and such.
     
 From (1), we have the Fact of an "EntityThroughoutTime", which is a partitioning of the time line into intervals, plus
   a mapping from each interval to a (constant throughout that interval) EntityData.
   This EntityData may describe a trajectory, so that the Entity represents a continuously moving object.
   
 (2) is never a hard limit; the time_steward will still function even if we must examine *every* entity to determine *any* entity's next change.
   Localizing the change function is simply a way to derive performance advantages.
 
 As a loosely described example, imagine a space full of moving circles that bounce off of each other.
   The Entities are the circles, and their EntityData is a radius and the coefficients of an affine function from time to circle center location.
   The Events are the moments of collision,
   and the time and nature of a Event can be derived from the EntityData of the nearby circles.
     (How we determine which circles to call 'nearby' is a complex matter better discussed elsewhere.)
   Suppose we have a group of circles that are all 'nearby' each other.
   For each pair, if those two are on a collision course for each other, it is a Fact that they will collide at a specific time if they remain on those courses.
   For various reasons, we choose to require the client code to represent that Fact as another Entity, which will exist in that form
     from the moment the two circles have assumed their current trajectories until the moment one of them diverts.
     To wit:

 Some Entities are persistent; others are temporary EventAnticipationEntities whose EntityData is simply a time and a description of Event.
   If a EventAnticipationEntity continues to exist until its time, then the Event happens.

 Fact:
   A time_steward (with given parameters) is defined entirely by
     (A) a single EntityData that is the initial state of a single object (called the "global object"), and
     (B) a collection of Events imposed by fiat, their existences and data contingent on nothing within the universe.
   We call A and B together a "Specification".
   It is a Fact that a Specification defines the entire history/future of a universe. Hence, given any Specification, there are
     a fixed (probably infinite) collection of Events that will happen.
     Data-wise, we only ever know of a finite collection of Events that might happen, *some* of which we know to actually happen.
   There are exactly two kinds of Events: Those imposed by fiat, which we will call FiatEvents,
     and those specified by EventAnticipationEntities, which we will call ConsequentialEvents.
   Other than the initial state of the global object, every EntityData is created by exactly one Event.
 
 A time_steward's external interface:
   Construct/initialize time_steward with a given global object EntityData.
   Add or remove one of the FiatEvents.
   Query the EntityData of an entity with a specific EntityID infinitesimally before or after a specific time.
     (To avoid confusion about the state at boundary moments, we never discuss "state AT a time", only infinitesimally before or after that time.)
   A few functions to manage resource usage.
 
 In code structure, we have:
   "External client code", such as UI code, creates a time_steward, and calls the external interface functions described above,
     passing in EventData, which are function objects.
   To answer the external client code's queries, the time_steward may call the EventData.
     We will refer to the EventData of ConsequentialEvents as "internal client code",
     and to the EventData of FiatEvents as "liminal client code",
       because it is subject to some of the restrictions of both internal and external client code.
     (These two kinds of Events are the only kinds of Events that exist.)
 
 (The global object exists to have a fixed EntityID as a point of reference for the client code.
   External and liminal client code aren't allowed to remember any other EntityIDs,
   because if a change is made in the past, those Entities might never exist.
   Instead, that code must ask for EntityData corresponding to the global object EntityID,
   and that EntityData may contain other EntityIDs.)
 
 A EventData is a function object that receives a single argument: A time_steward Accessor object tied to a specific time.
   It then uses the Accessor
     to create Entities, for which the Accessor will provide new EntityIDs,
     and/or
     to modify some Entities, which, internally, means marking that their current EntityData ends at this time
       and the modified EntityData begins at this time. Since EntityData is always copied, any large Entity should either
       use persistent data structures or be broken up into multiple Entities.
   EventData can only base its behavior on information from the Accessor and its own fields (information it was constructed with).
   The Accessor allows it to query the EntityData of any EntityID infinitesimally-before its time,
     and it tracks which EntityIDs are so queried, so that, even if external client code changes the past, the time_steward might not
     need to call this EventData again - if the change in the past doesn't affect any of the queried EntityData,
     then the time_steward knows that this EventData will give the same result as before, so the time_steward
     can keep using that result.
   A FiatEvent, as mentioned above, cannot know any EntityIDs except the global object ID, except through the Accessor.
   A ConsequentialEvent may know any EntityIDs that were known by the earlier Event that created this one's EventAnticipationEntity.
     (Those EntityIDs were meaningful at the time, and if a past change undoes the creation of one of them,
     then the earlier Event would never have known it either. If internal or liminal code changes an Entity in a way
     that would make this Event consider it invalid, that code is responsible for cleaning up the references to that Entity.
     Events cannot destroy an Entity per se, but they can replace it with a null EntityData and remove all references to it.)
 
 How does the time_steward work internally?
   If the time_steward knows (Fact-wise) all the EntityData at a given time A, then it knows all the EventAnticipationEntities.
     Thus it knows both the corresponding ConsequentialEvents and all the FiatEvents after A.
   Among those, one comes first, at a time B.
   No other Event exists that could change the Entities from (their state at A) before B.
   So the time_steward effectively "knows" the EntityData at all times in (A, B).
   Then it can apply the Event at B to those EntityData, which yields the state immediately after B - from B to whenever the next change is.
   This is part of how the process works Data-wise, too. There is a moving time to which the time_steward is updated,
     which is defined to be the time of the first Event that the time_steward knows about but hasn't processed yet - hereafter ExposedEvents.
   Hence, when someone inserts a FiatEvent far in the past, that FiatEvent becomes the first ExposedEvent,
     and hence, the time_steward knows that it "doesn't know for sure" any EntityData after that point.
     
 The time_steward may also have some "provisional knowledge" - EntityData etc. that it's computed beyond the updated-to-time.
   This can happen either directly (the time_steward processes a Event that's not the first),
     or indirectly (a FiatEvent is added prior to the first ExposedEvent).
   Provisional knowledge may be invalidated by changes that come later code-wise but earlier in-world.
   To see how we do this, consider a digraph where the nodes are EntityDataIntervals and Events.
     A Event that creates a new EntityData (and thus begins an EntityDataInterval) flows to that EntityDataInterval.
     A Event that *references* an Entity flows *from* the EntityDataInterval into which it's referencing.
   Whenever an EntityData is changed at a certain time, we undo all changes downstream of it in the digraph.
   This isn't quite accurate - if an entity is changed in the middle of an interval, we only need to invalidate references that
     came *after* that middle time. But the basic idea is there.
   
 The above assumes that Event times are unique (and totally ordered).
   We choose to enforce that fact because it makes work simpler for the client code.
   In order to do this, we consider two kinds of times: BaseTimes, which client code gives us;
     and ExtendedTimes, which are a BaseTime plus a distinguisher we create to order them even if their BaseTime is the same.
   This isn't a trivial task, because the ordering we create must be the same regardless of the order we processed the Events
     that brought us to this situation.
   To accomplish this, we use hashing. Each time and each entity gets a 128-bit unique ID using siphash.
     The global object EntityID is 0.
     Any other EntityID is hash(TimeID of when it was created, index among Entities created at that time).
     The TimeID of a ConsequentialEvent is hash(its BaseTime, the EntityID of its EventAnticipationEntity).
     The TimeID of a          FiatEvent is hash(its BaseTime, a distinguisher provided by the external client code).
 
 Some possibilities for improvement:
 - Have a way to discard some of the old data to save memory (at the price of needing extra processing time if/when we go back.)
 - Make time_steward itself a persistent data structure.
 
 
 
 
 
 When an event happens:
 
 A callback determines entity field accesses, entity field changes, and trigger changes.
 Triggers watching the changed fields also trigger.
 
 List of all past information contributing to this (we must catch every change in these things to update what happens):
 The state of each accessed field
 Whether each trigger was watching each modified field
 If this is a trigger, what its event func is
 
 List of all future information changed by this:
 The state of each changed field
 If this is a trigger, what fields it's watching
 For each changed trigger, what its event func is
 
 
 
 */

struct container_size_tracker {
  std::vector<int64_t> data;
  template<class C> void track(C const& c) {
    if (c.size() >= data.size()) {
      data.resize(c.size()+1);
    }
    ++data[c.size()];
  }
  void dump() {
    size_t b = 0;
    int64_t v = data[0];
    std::vector<size_t> halves;
    halves.push_back(0);
    for (size_t i = 1; i < data.size(); ++i) {
      if (data[i] <= data[halves.back()] / 2) {
        halves.push_back(i);
      }
    }
    for (size_t i = 0; i <= data.size(); ++i) {
      if (i == data.size() || data[i] != v) {
        if (b == i-1) {
          std::cerr << i-1 << ": " << data[i-1] << "\n";
        }
        else {
          std::cerr << b << "-" << i-1 << ": " << data[i-1] << "\n";
        }
        b = i;
        v = data[i];
      }
    }
    std::cerr << "halves\n";
    for (auto i : halves) {
      std::cerr << i << "\n";
    }
  }
  ~container_size_tracker() {
    dump();
  }
};

namespace time_steward_system {
#ifdef TRACKER_A
container_size_tracker tracker_a;
#endif

typedef siphash_id entity_id;
const entity_id global_object_id = siphash_id::least();

typedef size_t field_base_id;
template<typename T>
using optional = boost::optional<T>;
using boost::none;

namespace fields_list_impl {
  class fields_list_nature_base {};
  class empty_fields_list_nature : public fields_list_nature_base {};
  class nonempty_fields_list_nature : public fields_list_nature_base {};
  class generic_input_nature : public fields_list_nature_base {};
  class fields_list_entry_nature : public fields_list_nature_base {};
  
  struct two { char c[2]; };
  template<class T> std::enable_if_t<std::is_base_of<fields_list_nature_base, typename T::fields_list_nature>::value, char> test(int);
  template<class T> two test(...);
  template<class T, bool is_specified> struct get_fields_list_nature_impl { typedef typename T::fields_list_nature type; };
  template<class T> struct get_fields_list_nature_impl<T, false> { typedef generic_input_nature type; };
  template<class T> struct get_fields_list_nature { typedef typename get_fields_list_nature_impl<T, sizeof(test<T>(0))==1>::type type; };
  
  template<typename T>
  struct default_field_traits {
    static inline T null() { return T(); }
    static inline bool is_null(T const& t) { return bool(t); }
  };
  template<typename T, typename Traits>
  struct null_const_ref {
    static inline T const& get() {
      static const T value = Traits::null();
      return value;
    }
  };
  
  template<typename IdentifyingType, typename InnerDataType = IdentifyingType, typename InnerDataTraits = default_field_traits<InnerDataType>, bool PerID = false> class fields_list_entry;
  template<typename IdentifyingType, typename InnerDataType, typename InnerDataTraits>
  class fields_list_entry<IdentifyingType, InnerDataType, InnerDataTraits, false> {
  public:
    typedef fields_list_entry_nature fields_list_nature;
    typedef IdentifyingType identifying_type;
    typedef InnerDataType inner_data_type;
    typedef InnerDataTraits traits;
    static const bool per_id = false;
  };
  template<typename IdentifyingType, typename InnerDataType, typename InnerDataTraits>
  class fields_list_entry<IdentifyingType, InnerDataType, InnerDataTraits, true> {
  public:
    typedef fields_list_entry_nature fields_list_nature;
    typedef IdentifyingType identifying_type;
    typedef InnerDataType inner_data_type;
    typedef InnerDataTraits traits;
    static const bool per_id = true;
  };
  template<typename FieldsListEntry, template<typename> class Data, bool Persistent, bool PerID = FieldsListEntry::per_id> struct fields_list_entry_data; 
  template<typename FieldsListEntry, template<typename> class Data, bool Persistent>
  struct fields_list_entry_data<FieldsListEntry, Data, Persistent, false> {
    typedef Data<typename FieldsListEntry::inner_data_type> type;
    type      & get()      { return t; }
    type const& get()const { return t; }
    type const* find()const { return &t; }
    template<class... Args>
    type& set(type& t, Args&&... args){ return t = type(std::forward<Args>(args)...); }
  private:
    type t;
  };
  template<typename FieldsListEntry, template<typename> class Data>
  struct fields_list_entry_data<FieldsListEntry, Data, true, true> {
    typedef Data<typename FieldsListEntry::inner_data_type> inner_type;
    typedef persistent_siphash_id_map<inner_type> type;
    inner_type const& get(siphash_id id)const {
      auto i = t.find(id);
      if (i) { return i->second; }
      return null_const_ref<typename FieldsListEntry::inner_data_type, typename FieldsListEntry::traits>::get();
    }
    template<class... Args>
    inner_type const& set(siphash_id id, Args&&... args) {
      auto i = t.find(id);
      t = t.emplace(id, std::forward<Args>(args)...);
      return i->second;
    }
  private:
    type t;
  };
  template<typename FieldsListEntry, template<typename> class Data>
  struct fields_list_entry_data<FieldsListEntry, Data, false, true> {
    typedef Data<typename FieldsListEntry::inner_data_type> inner_type;
    typedef std::unordered_map<siphash_id, inner_type> type;
    inner_type& get(siphash_id id) {
      return t[id];
    }
    inner_type const* find(siphash_id id)const {
      auto i = t.find(id);
      if (i != t.end()) { return &i->second; }
      return nullptr;
    }
  private:
    type t;
  };
  
  template<typename Input, typename InputNature> struct make_fields_list_entry;
  template<typename Input> struct make_fields_list_entry<Input, generic_input_nature> { typedef fields_list_entry<Input, optional<Input>> type; };
  template<typename Input> struct make_fields_list_entry<Input, fields_list_entry_nature> { typedef Input type; };
  
  template<typename FieldsList, typename FieldID> struct get_field_entry;
  template<typename FieldsList, typename FieldID, bool Same> struct get_field_entry_impl;
  template<typename FieldsList, typename FieldID> struct get_field_entry_impl<FieldsList, FieldID, true> { typedef typename FieldsList::head type; };
  template<typename FieldsList, typename FieldID> struct get_field_entry_impl<FieldsList, FieldID, false> { typedef typename get_field_entry<typename FieldsList::tail, FieldID>::type type; };
  template<typename FieldsList, typename FieldID> struct get_field_entry {
    typedef typename get_field_entry_impl<FieldsList, FieldID, std::is_same<FieldID, typename FieldsList::head::identifying_type>::value>::type type;
  };
  template<typename FieldsList, typename FieldID>
  using field_entry = typename get_field_entry<FieldsList, FieldID>::type;
  template<typename FieldsList, typename FieldID>
  using field_data = typename field_entry<FieldsList, FieldID>::inner_data_type;
  
  template<typename ...Input> struct make_fields_list;
  template<typename HeadNature, typename Head, typename ...Tail> struct make_fields_list_contents;
  
  class empty_fields_list {
  public:
    typedef empty_fields_list_nature fields_list_nature;
    static const size_t size = 0;
    //template<typename T> static constexpr field_base_id idx_of() { static_assert(false, "No field of that type exists"); }
    // TODO find a way to make invalid-field errors more readable
  };
  template<typename Head, typename ...Tail>
  class nonempty_fields_list {
  public:
    typedef nonempty_fields_list_nature fields_list_nature;
    typedef typename make_fields_list_entry<Head, typename get_fields_list_nature<Head>::type>::type head;
    typedef typename make_fields_list<Tail...>::type tail;
    static const field_base_id idx = tail::size;
    static const size_t size = tail::size + 1;
    template<typename T> static constexpr std::enable_if_t< std::is_same<T, typename head::identifying_type>::value, field_base_id> idx_of() { return idx; }
    template<typename T> static constexpr std::enable_if_t<!std::is_same<T, typename head::identifying_type>::value, field_base_id> idx_of() { return tail::template idx_of<T>(); }
    template<typename T> static inline std::enable_if_t< std::is_same<T, typename head::identifying_type>::value, field_data<nonempty_fields_list, T> const&>
    get_null_const_ref() { return null_const_ref<typename head::inner_data_type, typename head::traits>::get(); }
    template<typename T> static inline std::enable_if_t<!std::is_same<T, typename head::identifying_type>::value, field_data<nonempty_fields_list, T> const&>
    get_null_const_ref() { return tail::template get_null_const_ref<T>(); }
  };
  
  template<typename HeadNature, typename Head, typename ...Tail>
  struct make_fields_list_contents                                             { typedef nonempty_fields_list<Head, Tail...> type; };
  template<typename Head, typename ...Tail>
  struct make_fields_list_contents<   empty_fields_list_nature, Head, Tail...> { typedef    empty_fields_list                type; };
  template<typename Head, typename ...Tail>
  struct make_fields_list_contents<nonempty_fields_list_nature, Head, Tail...> {
    typedef typename make_fields_list<typename Head::head, typename Head::tail, Tail...>::type type;
  };
  
  template<>
  struct make_fields_list<> { typedef empty_fields_list type; };
  template<typename Head, typename ...Tail>
  struct make_fields_list<Head, Tail...> { typedef typename make_fields_list_contents<typename get_fields_list_nature<Head>::type, Head, Tail...>::type type; };

  template<typename T> using identity = T;
  template<class FieldsList, bool Persistent, template<typename> class repeated = identity>
  class foreach_field {
    typedef typename FieldsList::head head;
    fields_list_entry_data<head, repeated, Persistent> head_;
    foreach_field<typename FieldsList::tail, Persistent, repeated> tail_;
  public:
    template<typename T, typename... Ext> inline std::enable_if_t< std::is_same<T, typename head::identifying_type>::value,
    repeated<field_data<FieldsList, T>>      &> get(Ext... ext)      { return head_.get(ext...); }
    template<typename T, typename... Ext> inline std::enable_if_t<!std::is_same<T, typename head::identifying_type>::value,
    repeated<field_data<FieldsList, T>>      &> get(Ext... ext)      { return tail_.template get<T>(ext...); }
    template<typename T, typename... Ext> inline std::enable_if_t< std::is_same<T, typename head::identifying_type>::value,
    repeated<field_data<FieldsList, T>> const*> find(Ext... ext)const { return head_.find(ext...); }
    template<typename T, typename... Ext> inline std::enable_if_t<!std::is_same<T, typename head::identifying_type>::value,
    repeated<field_data<FieldsList, T>> const*> find(Ext... ext)const { return tail_.template find<T>(ext...); }
    template<typename T, typename... Args> inline std::enable_if_t< std::is_same<T, typename head::identifying_type>::value && !field_entry<FieldsList, T>::per_id,
    repeated<field_data<FieldsList, T>> const&> set(                  Args&&... args) { return head_.set(head_,       std::forward<Args>(args)...); }
    template<typename T, typename... Args> inline std::enable_if_t<!std::is_same<T, typename head::identifying_type>::value && !field_entry<FieldsList, T>::per_id,
    repeated<field_data<FieldsList, T>> const&> set(                  Args&&... args) { return tail_.template set<T, Args...>(       std::forward<Args>(args)...); }
    template<typename T, typename... Args> inline std::enable_if_t< std::is_same<T, typename head::identifying_type>::value &&  field_entry<FieldsList, T>::per_id,
    repeated<field_data<FieldsList, T>> const&> set(siphash_id which, Args&&... args) { return head_.set(head_, which, std::forward<Args>(args)...); }
    template<typename T, typename... Args> inline std::enable_if_t<!std::is_same<T, typename head::identifying_type>::value &&  field_entry<FieldsList, T>::per_id,
    repeated<field_data<FieldsList, T>> const&> set(siphash_id which, Args&&... args) { return tail_.template set<T, Args...>(which, std::forward<Args>(args)...); }
  };
  template<bool Persistent, template<typename> class repeated>
  class foreach_field<empty_fields_list, Persistent, repeated> {};
  
  template<class FieldsList, template<typename> class repeated = identity>
  class foreach_field_base {
    typedef typename FieldsList::head head;
    repeated<typename head::inner_data_type> head_;
    foreach_field_base<typename FieldsList::tail, repeated> tail_;
  public:
    template<typename T> inline std::enable_if_t< std::is_same<T, typename head::identifying_type>::value,
    field_data<FieldsList, T>      &> get()      { return head_; }
    template<typename T> inline std::enable_if_t<!std::is_same<T, typename head::identifying_type>::value,
    field_data<FieldsList, T>      &> get()      { return tail_.template get<T>(); }
    template<typename T> inline std::enable_if_t< std::is_same<T, typename head::identifying_type>::value,
    field_data<FieldsList, T> const&> get()const { return head_; }
    template<typename T> inline std::enable_if_t<!std::is_same<T, typename head::identifying_type>::value,
    field_data<FieldsList, T> const&> get()const { return tail_.template get<T>(); }
  };
  template<template<typename> class repeated>
  class foreach_field_base<empty_fields_list, repeated> {};
  
  template<class FieldsList, typename repeated>
  class foreach_field_base_array : public std::array<repeated, FieldsList::size> {
  public:
    template<typename T> inline repeated& get() { return (*this)[FieldsList::template idx_of<T>()]; }
    template<typename T> inline repeated const& get()const { return (*this)[FieldsList::template idx_of<T>()]; }
  };
  
  template<class FieldsList, class FuncClass>
  using func_type = decltype(&FuncClass::template func<typename FieldsList::head::identifying_type>);
  template<class FieldsList, class FuncClass>
  class function_array : public foreach_field_base_array<FieldsList, func_type<FieldsList, FuncClass>> {
  public:
    function_array() { add_fptr<FieldsList>(); }
  private:
    template<class SubList> inline std::enable_if_t<!std::is_same<SubList, empty_fields_list>::value> add_fptr() {
      typedef typename SubList::head::identifying_type T;
      foreach_field_base_array<FieldsList, func_type<FieldsList, FuncClass>>::template get<T>() = &FuncClass::template func<T>;
      add_fptr<typename SubList::tail>();
    }
    template<class SubList> inline std::enable_if_t< std::is_same<SubList, empty_fields_list>::value> add_fptr() {}
  };
}
template<typename ...Input> using fields_list = typename fields_list_impl::make_fields_list<Input...>::type;
template<typename IdentifyingType, typename InnerDataType = optional<IdentifyingType>, typename InnerDataTraits = fields_list_impl::default_field_traits<InnerDataType>>
using field        = fields_list_impl::fields_list_entry<IdentifyingType, InnerDataType, InnerDataTraits, false>;
template<typename IdentifyingType, typename InnerDataType = optional<IdentifyingType>, typename InnerDataTraits = fields_list_impl::default_field_traits<InnerDataType>>
using field_per_id = fields_list_impl::fields_list_entry<IdentifyingType, InnerDataType, InnerDataTraits, true>;
template<typename FieldsList, typename FieldID>
using field_data = fields_list_impl::field_data<FieldsList, FieldID>;

struct field_id {
  field_id(field_base_id base):base(base),which(){}
  field_id(field_base_id base, siphash_id which):base(base),which(which){}
  field_base_id base;
  siphash_id which;
  bool operator==(field_id const& o)const { return (base == o.base) && (which == o.which); }
};

struct entity_field_id {
  entity_field_id(entity_id e, field_id f):e(e),f(f){}
  entity_id e;
  field_id f;
  bool operator==(entity_field_id const& o)const { return (e == o.e) && (f == o.f); }
};

} //end namespace time_steward_system
namespace std {
  template<>
  struct hash<time_steward_system::entity_field_id> {
    public:
    size_t operator()(time_steward_system::entity_field_id const& i)const {
      return std::hash<siphash_id>()(i.e) ^ std::hash<siphash_id>()(i.f.which) ^ i.f.base;
    }
  };
}
namespace time_steward_system {

template<typename TimeSteward>
class time_steward_accessor {
private:
  typedef typename TimeSteward::entity_fields entity_fields;
  typedef typename TimeSteward::extended_time extended_time;
  typedef typename TimeSteward::time_type time_type;
  typedef typename TimeSteward::event event;
  typedef typename TimeSteward::trigger trigger;
  static const time_type never = TimeSteward::never;
  
  struct field_metadata {
    bool accessed_preexisting_state;
    bool ever_modified;
    bool accessed(time_steward_accessor const& acc, entity_field_id const& id) {
      // If we already overwrote a field,
      // then this is not accessing the field's state *before* this time.
      if (!ever_modified && !accessed_preexisting_state) {
        accessed_preexisting_state = true;
        acc.entity_fields_preexisting_state_accessed.push_back(id);
        return true;
      }
      return false;
    }
    void modified(time_steward_accessor const& acc, entity_field_id const& id) {
      if (!ever_modified) {
        ever_modified = true;
        acc.entity_fields_modified.push_back(id);
      }
    }
  };
  
  template<typename FieldData>
  struct field_info {
    FieldData value;
    field_metadata metadata;
  };
    
  struct entity_info {
    entity_id id;
    fields_list_impl::foreach_field<entity_fields, false, field_info> fields;
  };
  
public:
  struct entity_ref {
  public:
    entity_ref():data(nullptr){}
    explicit operator bool()const { return bool(data); }
    entity_id id()const { return data->id; }
  private:
    entity_info* data;
    entity_ref(entity_info* data):data(data){}
    friend class time_steward_accessor;
  };
  
private:
  template<typename FieldID, typename... Ext>
  inline field_data<entity_fields, FieldID>& get_impl(entity_ref e, bool modified, Ext... ext)const {
    const field_id idx(entity_fields::template idx_of<FieldID>(), ext...);
    auto& f = e.data->fields.template get<FieldID>(ext...);
    auto& result = f.value;
    const entity_field_id id(e.id(), idx);
    if (f.metadata.accessed(*this, id)) {
      result = ts_->template get_provisional_entity_field_before<FieldID>(e.id(), time_, ext...);
    }
    if (modified) { f.metadata.modified(*this, id); }
    return result;
  }
  template<typename FieldID, typename... Ext>
  inline field_data<entity_fields, FieldID>& set_impl(entity_ref e, field_data<entity_fields, FieldID> new_contents, Ext... ext) {
    const field_id idx(entity_fields::template idx_of<FieldID>(), ext...);
    auto& f = e.data->fields.template get<FieldID>(ext...);
    auto& result = f.value;
    const entity_field_id id(e.id(), idx);
    f.metadata.modified(*this, id);
    result = new_contents;
    return result;
  }
  
public:
  inline entity_ref get(entity_id id)const {
    entity_info& e = entities[id];
    e.id = id;
    return entity_ref(&e);
  }
  
  template<typename FieldID, typename... Ext>
  inline field_data<entity_fields, FieldID> const& get    (entity_ref e, Ext... ext)const { return get_impl<FieldID>(e, false, ext...); }
  template<typename FieldID, typename... Ext>
  inline field_data<entity_fields, FieldID>      & get_mut(entity_ref e, Ext... ext)      { return get_impl<FieldID>(e,  true, ext...); }
  template<typename FieldID>
  inline field_data<entity_fields, FieldID>& set(entity_ref e,                   field_data<entity_fields, FieldID> new_contents) { return set_impl<FieldID>(e, new_contents       ); }
  template<typename FieldID>
  inline field_data<entity_fields, FieldID>& set(entity_ref e, siphash_id which, field_data<entity_fields, FieldID> new_contents) { return set_impl<FieldID>(e, new_contents, which); }
  
  void anticipate_event(time_type when, std::shared_ptr<const event> e) {
    if (when == never) { return; }
    caller_correct_if(when >= time_->base_time, "You can't anticipate an event in the past");
    const extended_time ext_when = (when == time_->base_time) ?
      TimeSteward::extended_time_manager::make_event_time(time_, create_id()) :
      TimeSteward::extended_time_manager::make_event_time(when , create_id());
    assert(ext_when > time_);
    new_upcoming_events.push_back(std::make_pair(ext_when, e));
  };
  void set_trigger(trigger_id id, std::shared_ptr<const trigger> t) {
    trigger_changes[id] = t;
  };
  trigger_id set_trigger(std::shared_ptr<const trigger> t) {
    trigger_id id = create_id();
    set_trigger(id, t);
    return id;
  };
  trigger_id this_trigger()const { return trigger_id_; }
  time_type now()const {
    return time_->base_time;
  }
  entity_ref create_entity() {
    // All entity IDs must be unique.
    // This function is one of the two ways that new entity IDs are created.
    // (The other is the fixed entity ID of the global object.)
    // As long as the other requirement is preserved - that
    // no two events are allowed to happen at extended_times with the same ID -
    // this will result in all unique entity IDs.
    return get(create_id());
  }
  uint64_t random_bits(uint32_t bits) {
    return rng_.random_bits(bits);
  }
    
  time_steward_accessor(time_steward_accessor const&) = delete;
  time_steward_accessor(time_steward_accessor&&) = delete;
  time_steward_accessor& operator=(time_steward_accessor const&) = delete;
  time_steward_accessor& operator=(time_steward_accessor&&) = delete;
private:
  TimeSteward const* ts_;
  extended_time time_;
  trigger_id trigger_id_;
  siphash_random_generator rng_;
  mutable std::unordered_map<entity_id, entity_info> entities;
  
  std::unordered_map<trigger_id, std::shared_ptr<const trigger>> trigger_changes;
  std::vector<std::pair<extended_time, std::shared_ptr<const event>>> new_upcoming_events;
  mutable std::vector<entity_field_id> entity_fields_modified; // hack - we COULD make this non-mutable, but get_impl is easier this way
  mutable std::vector<entity_field_id> entity_fields_preexisting_state_accessed;

  siphash_id create_id() { return rng_.random_id(); }
    
  time_steward_accessor(TimeSteward const* ts, extended_time time, trigger_id trigger_id_ = trigger_id::null())
    :ts_(ts),time_(time),trigger_id_(trigger_id_),rng_(time_->id) {
    TimeSteward::extended_time_manager::claim(time_);
  }
public:
  ~time_steward_accessor() {
    TimeSteward::extended_time_manager::release(time_);
  }
private:
  void process_event(event const* e) {
    (*e)(this);
  }
  friend TimeSteward;
};


template<class FieldsList/*, class Event = */, class TimeTypeInfo = time_type_info<>>
class time_steward {
public:
  typedef FieldsList entity_fields;
  typedef time_steward_accessor<time_steward> accessor;
  friend class time_steward_accessor<time_steward>;
  
  typedef typename TimeTypeInfo::time_type time_type;
  static const time_type min_time = TimeTypeInfo::min_time;
  static const time_type max_time = TimeTypeInfo::max_time;
  static const time_type never = TimeTypeInfo::never;
  struct time_field_traits {
    static inline time_type null() { return never; }
    static inline bool is_null(time_type const& t) { return t == null(); }
  };
  typedef extended_time_maker<TimeTypeInfo> extended_time_manager;
  typedef typename extended_time_manager::extended_time extended_time;
  
  // TODO can the action/event/trigger stuff be template parameters rather than always being virtual classes to be subclassed?
  class event {
  public:
    virtual void operator()(accessor* accessor)const = 0;
    virtual ~event(){}
  };
  typedef event trigger;
private:
//   class nothing_happens : public event {
//   public:
//     void operator()(accessor*)const override {}
//   };
//   static std::shared_ptr<const event> const& nothing_happens_ptr() {
//     static std::shared_ptr<const event> a(new nothing_happens()); return a;
//   }
  typedef typename TimeTypeInfo::combine_hash_with_time_type_func_type combine_hash_with_time_type_func_type;
  
  enum execution_or_unexecution { EXECUTION = 0, UNEXECUTION = 1 };
  struct event_pile_info {
    event_pile_info(std::shared_ptr<const event> instigating_event, trigger_id tid = trigger_id::null()) :
      instigating_event (instigating_event),
      tid(tid),
      creation_cut_off (false),
      has_been_executed (false)
      {}
    std::shared_ptr<const event> instigating_event;
    trigger_id tid;
    // TODO: these can just be runtime-sized arrays
    std::vector<entity_field_id> entity_fields_pile_accessed;
    std::vector<entity_field_id> entity_fields_pile_modified;
    std::vector<trigger_id> triggers_changed;
    std::vector<extended_time> anticipated_events;
    size_t first_anticipated_event_idx_after(extended_time t)const {
      int64_t min = -1;
      int64_t max = anticipated_events.size();
      while(min + 1 < max) {
        const int64_t mid = (min+max)>>1;
        if (anticipated_events[mid] <= t) { min = mid; }
        else                              { max = mid; }
      }
      
      if (max == int64_t(anticipated_events.size())) {
        if (min >= 0) {
          assert (anticipated_events.back() < t);
        }
      }
      else {
        assert (anticipated_events[max] > t);
        if (min >= 0) {
          assert (anticipated_events[min] < t);
        }
      }
      
      return max;
    }
    bool creation_cut_off;
    bool has_been_executed;
    bool needs_unexecution;
    bool should_be_executed()const { return instigating_event && !creation_cut_off; }
  };
  
  struct field_metadata_throughout_time {
    bidirectional_interval_map<trigger_id, extended_time> triggers_pointing_at_this;
    std::set<extended_time> event_piles_which_accessed_this;
  };
  template<typename FieldData>
  using field_throughout_time = std::map<extended_time, FieldData>;
  template<typename FieldData>
  using field_info_by_entity = std::unordered_map<entity_id, field_throughout_time<FieldData>>;
//   struct trigger_call_info {
//     trigger_call_info():field_changes_andor_creations_triggering_this(0),replaced(false){}
//     uint32_t field_changes_andor_creations_triggering_this;
//     bool replaced;
//   };
//   typedef std::map<extended_time, trigger_call_info> trigger_throughout_time_info;
  struct trigger_call_info {
    trigger_call_info():field_changes_andor_creations_triggering_this(0){}
    uint32_t field_changes_andor_creations_triggering_this;
  };
  struct trigger_throughout_time_info {
    std::map<extended_time, trigger_call_info> calls;
    std::map<extended_time, std::shared_ptr<const event>> impositions;
    
    std::shared_ptr<const event> eventfunc_active_before(extended_time t)const {
      auto next = impositions.lower_bound(t);
      return (next == impositions.begin()) ? nullptr : boost::prior(next)->second;
    }
  };
  
  template<typename FieldID, typename... Ext>
  field_throughout_time<field_data<entity_fields, FieldID>>& get_field_throughout_time(entity_id const& id, Ext... ext) {
    return field_values.template get<FieldID>(ext...)[id];
  }
  template<typename FieldID>
  std::enable_if_t<!fields_list_impl::field_entry<entity_fields, FieldID>::per_id, field_throughout_time<field_data<entity_fields, FieldID>>&>
  get_field_throughout_time(entity_field_id const& id) {
    return get_field_throughout_time<FieldID>(id.e            );
  }
  template<typename FieldID>
  std::enable_if_t< fields_list_impl::field_entry<entity_fields, FieldID>::per_id, field_throughout_time<field_data<entity_fields, FieldID>>&>
  get_field_throughout_time(entity_field_id const& id) {
    return get_field_throughout_time<FieldID>(id.e, id.f.which);
  }
  field_metadata_throughout_time& require_field_metadata_throughout_time(entity_field_id const& id) {
#ifdef TRACKER_A
    auto i = field_metadata.find(id);
#endif
    field_metadata_throughout_time& result = field_metadata[id];
#ifdef TRACKER_A
    if (i == field_metadata.end()) {
      tracker_a.track(result.event_piles_which_accessed_this);
    }
#endif
    return result;
  }
  field_metadata_throughout_time const& get_field_metadata_throughout_time(entity_field_id const& id)const {
    auto i = field_metadata.find(id);
    assert (i != field_metadata.end());
    return i->second;
  }
  field_metadata_throughout_time& get_mut_field_metadata_throughout_time(entity_field_id const& id) {
    auto i = field_metadata.find(id);
    assert (i != field_metadata.end());
    return i->second;
  }
  
  typedef std::unordered_map<entity_field_id, field_metadata_throughout_time> field_metadata_map;
  // The events map doesn't need to be ordered (even though it has a meaningful order)
  // because it's just for looking up events whose times we know.
  typedef std::unordered_map<extended_time, event_pile_info> event_piles_map;
  
  fields_list_impl::foreach_field<entity_fields, false, field_info_by_entity> field_values; // TODO rename this to be more explanatory
  field_metadata_map field_metadata;
  event_piles_map event_piles;
//   std::set<extended_time> event_piles_needing_execution;
//   std::set<extended_time> event_piles_needing_unexecution;
  std::unordered_map<trigger_id, trigger_throughout_time_info> triggers;
  mutable std::array<std::priority_queue<extended_time, std::vector<extended_time>, std::greater<extended_time> >, 2> event_piles_needing;
  extended_time next_event_needing(execution_or_unexecution e)const {
    while (true) {
      if (event_piles_needing[e].empty()) { return extended_time_manager::max_time(); }
      extended_time t = event_piles_needing[e].top();
      auto i = event_piles.find(t);
      if (i != event_piles.end()) {
        if (e == EXECUTION) {
          if (i->second.should_be_executed() && !i->second.has_been_executed) {
            return t;
          }
        }
        else {
          if (i->second.needs_unexecution) {
            assert (i->second.has_been_executed);
            return t;
          }
        }
      }
      event_piles_needing[e].pop();
      extended_time_manager::release(t);
    }
  }
  void event_needs(execution_or_unexecution e, extended_time t) {
    extended_time_manager::claim(t);
    auto i = event_piles.find(t);
    assert (i != event_piles.end());
    if (e == EXECUTION) {
      assert (i->second.should_be_executed() && !i->second.has_been_executed);
    }
    else {
      assert (i->second.has_been_executed);
      //assert (!i->second.needs_unexecution);
      i->second.needs_unexecution = true;
    }
    event_piles_needing[e].push(t);
  }
  
  void cut_off_trigger_events(bool undoing, trigger_id tid, extended_time trigger_call_time) {
    const auto cut_off_call_ptr = last_executed_trigger_call(tid, trigger_call_time);
    if (cut_off_call_ptr) {
      const auto next_call_ptr = next_executed_trigger_call(tid, trigger_call_time);
      const auto cut_off_pile_iter = event_piles.find(cut_off_call_ptr->first);
      assert(cut_off_pile_iter != event_piles.end());
      auto const& cut_off_pile_info = cut_off_pile_iter->second;
      for (size_t i = cut_off_pile_info.first_anticipated_event_idx_after(trigger_call_time); // upper vs lower shouldn't matter: triggers are never anticipated events
                i != cut_off_pile_info.anticipated_events.size(); ++i) {
        const extended_time t = cut_off_pile_info.anticipated_events[i];
        const auto pile_iter = event_piles.find(t);
        assert(pile_iter != event_piles.end());
        auto& pile_info = pile_iter->second;
        if (next_call_ptr) {
          assert (t != next_call_ptr->first); // > vs >= shouldn't matter: anticipated events are never triggers
          if (t > next_call_ptr->first) {
            // We can skip these because they are cut off by the later trigger regardless.
            assert (pile_info.creation_cut_off == true);
            // TODO uncomment:
            //break;
            continue;
          }
        }
        if (undoing) {
          assert (pile_info.creation_cut_off == true);
          pile_info.creation_cut_off = false;
          assert (pile_info.should_be_executed());
          if (pile_info.has_been_executed) { /*could've been invalidated, so no. event_piles_needing_unexecution.erase (t);*/ }
          else                             { event_needs(EXECUTION, t); }
        }
        else {
          assert (pile_info.creation_cut_off == false);
          pile_info.creation_cut_off = true;
          if (pile_info.has_been_executed) { event_needs(UNEXECUTION, t); }
          //else                             { event_piles_needing_execution  .erase (t); }
        }
      }
    }
  }
    
  void impose_trigger_eventfunc(bool undoing, trigger_throughout_time_info& trigger_info, extended_time t, std::shared_ptr<const event> new_eventfunc = nullptr) {
    auto next = trigger_info.impositions.upper_bound(t);
    std::shared_ptr<const event> old_eventfunc = (next == trigger_info.impositions.begin()) ? nullptr : boost::prior(next)->second;
    if (undoing) {
      assert (next != trigger_info.impositions.begin());
      auto l = boost::prior(next);
      assert (l->first == t);
      if (l == trigger_info.impositions.begin()) { new_eventfunc = nullptr; }
      else                          { new_eventfunc = boost::prior(l)->second; }
      trigger_info.impositions.erase(l);
    }
    else {
      auto p = trigger_info.impositions.insert(std::make_pair(t, new_eventfunc));
      assert (p.second);
    }
    
    extended_time next_time = (next == trigger_info.impositions.end()) ? extended_time_manager::max_time() : next->first;
    if (new_eventfunc != old_eventfunc) {
      for (auto i = trigger_info.calls.upper_bound(t); (i != trigger_info.calls.end()) && (i->first <= next_time); ++i) {
        const auto pile_iter = event_piles.find(i->first);
        assert (pile_iter != event_piles.end());
        if (pile_iter->second.instigating_event) {
          //assert (trigger_info.eventfunc_active_before(i->first) == old_eventfunc);
          assert (pile_iter->second.instigating_event == old_eventfunc);
          if (new_eventfunc) {
            pile_iter->second.instigating_event = new_eventfunc;
            if (pile_iter->second.has_been_executed) { event_needs(UNEXECUTION, pile_iter->first); }
          }
          else {
            erase_instigating_event(i->first);
          }
        }
      }
    }
  }
  void update_trigger(bool undoing, trigger_id tid, extended_time field_change_andor_creation_time) {
    // with undoing==false: Either a trigger was created/replaced, or an entity a trigger-call accessed changed.
    //   We need to do two things:
    //   1) Create a new trigger-call soon after the current time (if it doesn't already exist);
    //   2) Cancel any events that trigger calls anticipated to happen after the current time (which should be equivalent to "after the new trigger time").
    // We do (2), then (1).
    // Note that with undoing==true, we need to do the parts in the opposite order (undo 1, then undo 2).
    trigger_throughout_time_info& trigger_info = triggers[tid];
    const extended_time new_trigger_call_time = extended_time_manager::make_trigger_call_time(tid, field_change_andor_creation_time);
    bool time_claimed = false;
    
    if (!undoing) {
      std::shared_ptr<const trigger> new_trigger = trigger_info.eventfunc_active_before(new_trigger_call_time);
      
      const auto p = trigger_info.calls.insert(std::make_pair(new_trigger_call_time, trigger_call_info()));
      if (p.second && new_trigger) {
        time_claimed = true;
        insert_instigating_event(new_trigger_call_time, event_pile_info(new_trigger, tid));
      }
      ++p.first->second.field_changes_andor_creations_triggering_this;
    }
    else {
      const auto i = trigger_info.calls.find(new_trigger_call_time);
      assert(i != trigger_info.calls.end());
      const auto pile_iter = event_piles.find(i->first);
      assert(pile_iter != event_piles.end());
      --i->second.field_changes_andor_creations_triggering_this;
      if (i->second.field_changes_andor_creations_triggering_this == 0) {
        erase_instigating_event(new_trigger_call_time);
      }
    }
    
    if (!time_claimed) {
      extended_time_manager::release(new_trigger_call_time);
    }
  }
  void update_triggers(bool undoing, field_metadata_throughout_time const& metadata, extended_time field_change_time) {
    for (auto v : metadata.triggers_pointing_at_this.range_containing(field_change_time)) {
      // bidirectional_interval_map acts as if its intervals are closed intervals but we treat them as right-open ones.
      // if a trigger checks a field and then changes it, it retriggers itself.
      // if it changes the field without checking it, it doesn't, even if its previous iteration checked that field.
      if (v.interval.bounds[1] > field_change_time) {
        update_trigger(undoing, v.key, field_change_time);
      }
    }
  }
  void invalidate_events(field_metadata_throughout_time const& metadata, extended_time field_change_time, extended_time field_change_end_time) {
    for (auto incorrect_event_time = metadata.event_piles_which_accessed_this.upper_bound(field_change_time); // upper_bound:
        // don't paradoxically cancel the event that changed this field
         (incorrect_event_time != metadata.event_piles_which_accessed_this.end()) && (*incorrect_event_time <= field_change_end_time); // <=:
         // if the event that changed this field examined this field, it needs redoing
         ++incorrect_event_time) {
      assert(*incorrect_event_time > field_change_time);
      event_needs(UNEXECUTION, *incorrect_event_time);
    }
  }
  template<typename Iterator, typename Map>
  void invalidate_events(field_metadata_throughout_time const& metadata, Iterator const& field_iter, Map const& f) {
    const Iterator next = boost::next(field_iter);
    invalidate_events(metadata, field_iter->first, (next == f.end()) ? extended_time_manager::max_time() : next->first);
  }
  
  struct copy_field_change_from_accessor {
    template<typename FieldID>
    static std::enable_if_t<!fields_list_impl::field_entry<entity_fields, FieldID>::per_id, field_data<entity_fields, FieldID>>
    get_field(decltype(accessor::entity_info::fields)& src, entity_field_id const&   ) {
      return src.template get<FieldID>(          ).value;
    }
    template<typename FieldID>
    static std::enable_if_t< fields_list_impl::field_entry<entity_fields, FieldID>::per_id, field_data<entity_fields, FieldID>>
    get_field(decltype(accessor::entity_info::fields)& src, entity_field_id const& id) {
      return src.template get<FieldID>(id.f.which).value;
    }
    
    template<typename FieldID>
    static void func(time_steward* ts, entity_field_id const& id, field_metadata_throughout_time const& metadata, extended_time time, decltype(accessor::entity_info::fields)& src) {
      auto& f = ts->get_field_throughout_time<FieldID>(id);
      const auto p = f.insert(std::make_pair(time, get_field<FieldID>(src, id)));
      assert(p.second);
      ts->invalidate_events(metadata, p.first, f);
    }
  };
  struct undo_field_change {
    template<typename FieldID>
    static void func(time_steward* ts, entity_field_id const& id, field_metadata_throughout_time const& metadata, extended_time time) {
      auto& f = ts->get_field_throughout_time<FieldID>(id);
      const auto i = f.find(time);
      assert(i != f.end());
      ts->invalidate_events(metadata, i, f);
      f.erase(i);
    }
  };
  
  // TODO: Theoretically, these could waste a lot of time if there are too many unexecuted trigger calls in between executed ones. Fix that.
  std::pair<const extended_time, trigger_call_info> const* next_executed_trigger_call(trigger_id tid, extended_time time)const {
    const auto trigger_info_iter = triggers.find(tid);
    if (trigger_info_iter == triggers.end()) { return nullptr; }
    auto& trigger_info = trigger_info_iter->second;
    auto next_call_iter = trigger_info.calls.upper_bound(time);
    while ((next_call_iter != trigger_info.calls.end()) && !event_piles.find(next_call_iter->first)->second.has_been_executed) {++next_call_iter;} // TODO: asymptotics?
    if (next_call_iter == trigger_info.calls.end()) { return nullptr; }
    return &*next_call_iter;
  }
  std::pair<const extended_time, trigger_call_info> const* last_executed_trigger_call(trigger_id tid, extended_time time)const {
    const auto trigger_info_iter = triggers.find(tid);
    if (trigger_info_iter == triggers.end()) { return nullptr; }
    auto& trigger_info = trigger_info_iter->second;
    auto last_call_iter = trigger_info.calls.lower_bound(time);
    while ((last_call_iter != trigger_info.calls.begin()) && !event_piles.find(boost::prior(last_call_iter)->first)->second.has_been_executed) {--last_call_iter;} // TODO: asymptotics?
    if (last_call_iter == trigger_info.calls.begin()) { return nullptr; }
    return &*boost::prior(last_call_iter);
  }
//   std::pair<const extended_time, trigger_call_info> const* next_scheduled_trigger_call(trigger_id tid, extended_time time)const {
//     const auto trigger_info_iter = triggers.find(tid);
//     if (trigger_info_iter == triggers.end()) { return nullptr; }
//     auto& trigger_info = trigger_info_iter->second;
//     auto next_call_iter = trigger_info.upper_bound(time);
//     //while ((next_call_iter != trigger_info.end()) && !event_piles.find(next_call_iter->first)->second.instigating_event) {++next_call_iter;} // TODO: asymptotics?
//     if (next_call_iter == trigger_info.end()) { return nullptr; }
//     return &*next_call_iter;
//   }
//   std::pair<const extended_time, trigger_call_info> const* last_scheduled_trigger_call(trigger_id tid, extended_time time)const {
//     const auto trigger_info_iter = triggers.find(tid);
//     if (trigger_info_iter == triggers.end()) { return nullptr; }
//     auto& trigger_info = trigger_info_iter->second;
//     auto last_call_iter = trigger_info.lower_bound(time);
//     //while ((last_call_iter != trigger_info.begin()) && !event_piles.find(boost::prior(last_call_iter)->first)->second.instigating_event) {--last_call_iter;} // TODO: asymptotics?
//     if (last_call_iter == trigger_info.begin()) { return nullptr; }
//     return &*boost::prior(last_call_iter);
//   }
//   std::pair<const extended_time, trigger_call_info> const* last_valid_trigger_call(trigger_id tid, extended_time time)const {
//     const auto trigger_info_iter = triggers.find(tid);
//     if (trigger_info_iter == triggers.end()) { return nullptr; }
//     auto& trigger_info = trigger_info_iter->second;
//     auto last_call_iter = trigger_info.lower_bound(time);
//     while ((last_call_iter != trigger_info.begin()) && !event_piles.find(boost::prior(last_call_iter)->first)->second.instigating_event) {--last_call_iter;} // TODO: asymptotics?
//     if (last_call_iter == trigger_info.begin()) { return nullptr; }
//     return &*boost::prior(last_call_iter);
//   }
  void delete_trigger_access_record(entity_field_id const& id, field_metadata_throughout_time& metadata, extended_time time, trigger_id tid) {
    update_trigger_access_record(id, metadata, time, tid, bool(), true);
  }
  void update_trigger_access_record(entity_field_id const& id, field_metadata_throughout_time& metadata, extended_time time, trigger_id tid, bool undoing, bool force_erase = false) {
    maybe_assert (event_piles.find(time)->second.tid == tid);
    
    auto next_call_ptr = next_executed_trigger_call(tid, time);
    const extended_time change_end = next_call_ptr ? next_call_ptr->first : extended_time_manager::max_time();
    
    if (force_erase) { undoing = !metadata.triggers_pointing_at_this.key_active_before(tid, time); }
    const bool changed = (undoing == metadata.triggers_pointing_at_this.key_active_after(tid, time));
    maybe_assert (metadata.triggers_pointing_at_this.key_active_after(tid, time) == metadata.triggers_pointing_at_this.key_active_before(tid, change_end));
    
    if (changed) {
      if (undoing) {
        metadata.triggers_pointing_at_this.erase (tid, time, change_end);
      }
      else {
        metadata.triggers_pointing_at_this.insert(tid, time, change_end);
      }
      // If we changed the future of metadata.triggers_pointing_at_this, 
      // anything in the future that referred to metadata.triggers_pointing_at_this
      // needs to be updated.
      (*update_future_trigger_calls_funcs[id.f.base])(this, id, time, change_end, tid, undoing);
    }
    maybe_assert (metadata.triggers_pointing_at_this.key_active_after(tid, time) == metadata.triggers_pointing_at_this.key_active_before(tid, change_end));
    if (force_erase) {
      maybe_assert (metadata.triggers_pointing_at_this.key_active_after(tid, time) == metadata.triggers_pointing_at_this.key_active_before(tid, time));
    }
  }
  struct update_future_trigger_calls {
    template<typename FieldID>
    static void func(time_steward* ts, entity_field_id const& id, extended_time change_start, extended_time change_end, trigger_id tid, bool undoing) {
      auto const& f = ts->get_field_throughout_time<FieldID>(id);
      for (auto i = f.upper_bound(change_start); i != f.end(); ++i) { // upper_bound: we handle f[time], if any, in the regular update_triggers.
        if (i->first >= change_end) { break; } // >=:
          // if a trigger changes a field, it doesn't matter what the trigger was looking at before that time.
        ts->update_trigger(undoing, tid, i->first);
      }
    }
  };
  
  // statics must also be declared outside the class like other globals
  //   but we can't figure out how to do that WRT templating (id U+cErimeRjHMjQ)
  /*static*/ const fields_list_impl::function_array<entity_fields, copy_field_change_from_accessor> copy_field_change_from_accessor_funcs;
  /*static*/ const fields_list_impl::function_array<entity_fields, undo_field_change> undo_field_change_funcs;
  /*static*/ const fields_list_impl::function_array<entity_fields, update_future_trigger_calls> update_future_trigger_calls_funcs;
public:
  time_steward() {}
  ~time_steward() {
    for (auto& e : event_piles) {
      extended_time_manager::release(e.first);
    }
  }
  
  void insert_fiat_event(time_type time, uint64_t distinguisher, std::shared_ptr<const event> e) {
    // This function is one of exactly two places where
    // (persistent) extended_times are created.
    // Uniqueness justification:
    // TODO
    const extended_time t = extended_time_manager::make_event_time(time, combine_hash_with_time_type_func_type()(siphash_id::combining(distinguisher), time));
    // TODO throw an exception if the user inserts two events at the same time with the same distinguisher
    auto i = event_piles.find(t);
    if (i != event_piles.end() && i->second.has_been_executed) {
      assert (!i->second.should_be_executed());
      const bool event_pile_deleted_for_being_out_of_date = unexecute_event_pile(t);
      assert (event_pile_deleted_for_being_out_of_date);
    }
    insert_instigating_event(t, event_pile_info(e));
  }
  void erase_fiat_event(time_type time, uint64_t distinguisher) {
    const auto t = typename extended_time_manager::scoped(extended_time_manager::make_event_time(time, combine_hash_with_time_type_func_type()(siphash_id::combining(distinguisher), time)));
    erase_instigating_event(t.get());
  }
  
  // For external client code to examine the state - 
  //   e.g., for display.
  // Output of these should never find their way into internal client code,
  //   and it's a little strange (but ok) to have them affect FiatEvents.
  // Any change to the time steward invalidates this accessor.
  /*template<typename T>
  entity_ref<T const> get_entity_data_after(entity_id<T> const& id, time_type const& time) {
    // Note: For collision safety, these extended_times must not persist.
    return dynamic_pointer_cast<T const>(entity_ref<entity const>(
      get_actual_entity_data_before(id, extended_time(time, extended_time::last))));
  }*/
  std::unique_ptr<accessor> accessor_after(time_type const& time) {
    const extended_time end_time = extended_time_manager::make_max_at(time);
    update_through_time_impl(time);
    auto temp = new accessor(this, end_time);
    extended_time_manager::release(end_time);
    return std::unique_ptr<accessor>(temp);
  }
  
  // Some functions for external client code to regulate how much processing the time_steward
  //   does in advance of when it's needed.
  void update_through_time(time_type const& time) {
    update_through_time_impl(time);
  }
  void debug__randomly_update_through_time(time_type const& time) {
    while (!is_updated_through(time)) {
      std::array<std::vector<extended_time>, 2> hidden;
      extended_time stop = next_event_needing(UNEXECUTION);
      for (int i = 0; i < 2; ++i) {
        next_event_needing(execution_or_unexecution(i));
        while (event_piles_needing[i].size() > 1 && (rand()&3)) {
          hidden[i].push_back(event_piles_needing[i].top());
          event_piles_needing[i].pop();
          next_event_needing(execution_or_unexecution(i));
        }
      }
      extended_time a = next_event_needing(EXECUTION);
      extended_time b = next_event_needing(UNEXECUTION);
      for (int i = 0; i < 2; ++i) {
        for (extended_time j : hidden[i]) {
          event_piles_needing[i].push(j);
        }
      }
      
      if (stop >= a && a < extended_time_manager::max_time() && (rand()&3)) {
        execute_event_pile(a);
      }
      else if (b < extended_time_manager::max_time()) {
        unexecute_event_pile(b);
      }
    }
  }
  void debug__check_equivalence(time_steward const& other)const {
    std::unordered_set<siphash_id> time_ids0;
    std::unordered_set<siphash_id> time_ids1;
    for (auto const& p : event_piles) {
      if (is_updated_through(p.first->base_time) && other.is_updated_through(p.first->base_time)) {
        time_ids0.insert(p.first->id);
      }
    }
    for (auto const& p : other.event_piles) {
      if (is_updated_through(p.first->base_time) && other.is_updated_through(p.first->base_time)) {
        time_ids1.insert(p.first->id);
      }
    }
    for (auto const& id : time_ids0) {
      assert(time_ids1.find(id) != time_ids1.end());
    }
    for (auto const& id : time_ids1) {
      assert(time_ids0.find(id) != time_ids0.end());
    }
    
    debug__check_equivalence_one_sided(other);
    other.debug__check_equivalence_one_sided(*this);
  }
private:
  void debug__check_equivalence_one_sided(time_steward const& /*other*/)const {
  }
  void validate()const {
    return;
//     for (auto const& t : event_piles_needing_execution) {
//       assert (event_piles.find(t) != event_piles.end());
//     }
//     for (auto const& t : event_piles_needing_unexecution) {
//       assert (event_piles.find(t) != event_piles.end());
//     }
    for (auto const& p : triggers) {
      for (auto const& p2 : p.second.calls) {
        const auto e = event_piles.find(p2.first);
        assert (e != event_piles.end());
        assert (e->second.tid == p.first);
      }
    }
    for (auto const& p : field_metadata) {
      for (auto const& p2 : p.second.event_piles_which_accessed_this) {
        const auto e = event_piles.find(p2);
        assert (e != event_piles.end());
        //assert (e->second.entity_fields_pile_accessed.find(p.first) != e->second.entity_fields_pile_accessed.end());
      }
    }
    for (auto const& p : event_piles) {
      if (p.second.has_been_executed) {
//         if (!p.second.should_be_executed()) {
//           assert (event_piles_needing_unexecution.find(p.first) != event_piles_needing_unexecution.end());
//           assert (event_piles_needing_execution.find(p.first) == event_piles_needing_execution.end());
//         }
      }
      else {
        assert (p.second.entity_fields_pile_accessed.empty());
        assert (p.second.entity_fields_pile_modified.empty());
        assert (p.second.triggers_changed.empty());
        assert (p.second.anticipated_events.empty());
//         if (p.second.should_be_executed()) {
//           assert (event_piles_needing_execution.find(p.first) != event_piles_needing_execution.end());
//         }
//         else {
//           assert (event_piles_needing_execution.find(p.first) == event_piles_needing_execution.end());
//         }
//        assert (event_piles_needing_unexecution.find(p.first) == event_piles_needing_unexecution.end());
      }
      for (auto et : p.second.anticipated_events) {
        const auto e = event_piles.find(et);
        assert (e != event_piles.end());
        if (!p.second.tid) {
          assert (!e->second.creation_cut_off);
        }
      }
      if (p.second.tid) {
        if (p.second.instigating_event) {
//           auto const& trigger_info = triggers.find(p.second.tid)->second;
//           auto const& t = trigger_info.find(p.first);
//           assert (t != trigger_info.end());
//           assert (t->second.field_changes_andor_creations_triggering_this > 0); // otherwise !p.second.instigating_event
//           // TODO: check that t->second.field_changes_andor_creations_triggering_this
//           // is actually the same as the number of things triggering it
//           const auto next_call_ptr = next_scheduled_trigger_call(p.second.tid, p.first);
//           for (auto et : p.second.anticipated_events) {
//             const auto e = event_piles.find(et);
//             if (next_call_ptr && (et > next_call_ptr->first)) {
//               assert (e->second.creation_cut_off);
//             }
//             else {
//               assert (!e->second.creation_cut_off);
//             }
//           }
        }
      }
      for (auto id : p.second.entity_fields_pile_accessed) {
        auto const& f = field_metadata.find(id)->second;
        assert (f.event_piles_which_accessed_this.find(p.first) != f.event_piles_which_accessed_this.end());
      }
      for (auto id : p.second.entity_fields_pile_modified) {
        auto const& f = field_metadata.find(id)->second;
        for (auto v : f.triggers_pointing_at_this.range_containing(p.first)) {
          // bidirectional_interval_map acts as if its intervals are closed intervals but we treat them as right-open ones.
          // if a trigger checks a field and then changes it, it retriggers itself.
          // if it changes the field without checking it, it doesn't, even if its previous iteration checked that field.
          if (v.interval.bounds[1] > p.first) {
            const auto when_trigger_called = typename extended_time_manager::scoped(extended_time_manager::make_trigger_call_time(v.key, p.first));
            const auto e = event_piles.find(when_trigger_called.get());
            assert (e != event_piles.end());
            assert (e->second.instigating_event);
          }
        }
        // TODO that the change exists (need another template func)
      }
      for (auto tid : p.second.triggers_changed) {
        const auto when_trigger_called = typename extended_time_manager::scoped(extended_time_manager::make_trigger_call_time(tid, p.first));
        const auto e = event_piles.find(when_trigger_called.get());
        assert (e != event_piles.end());
        assert (e->second.instigating_event);
      }
    }
  }
  
  void update_through_time_impl(time_type const& time) {
    while (!is_updated_through(time)) {
      extended_time a = next_event_needing(EXECUTION);
      extended_time b = next_event_needing(UNEXECUTION);
      if (b < a) {
        unexecute_event_pile(b);
      }
      else {
        assert(a != extended_time_manager::max_time());
        execute_event_pile(a);
      }
    }
  }
  bool is_updated_through(time_type const& time)const {
    extended_time a = next_event_needing(EXECUTION);
    extended_time b = next_event_needing(UNEXECUTION);
    return (a->base_time > time) && (b->base_time > time);
  }
  template<typename FieldID, typename... Ext>
  field_data<entity_fields, FieldID> const& get_provisional_entity_field_before(entity_id id, extended_time time, Ext... ext)const {
    auto const* fd = field_values.template find<FieldID>(ext...);
    if (fd) {
      const auto e = fd->find(id);
      if (e != fd->end()) {
        field_throughout_time<field_data<entity_fields, FieldID>> const& f = e->second;
        
        const auto next_change_iter = f.lower_bound(time);
        if (next_change_iter!= f.begin()) {
          return boost::prior(next_change_iter)->second;
        }
      }
    }
    
    return entity_fields::template get_null_const_ref<FieldID>();
  }
  
  void insert_instigating_event(extended_time time, event_pile_info const& e) {
    /*if (event_piles_needing_unexecution.find(time) != event_piles_needing_unexecution.end()) {
      // TODO: is it safe to unexecute an event in the future during execute_event_pile?
      const bool event_pile_deleted_for_being_out_of_date = unexecute_event_pile(time);
      assert (event_pile_deleted_for_being_out_of_date);
    }*/
    
    const auto p1 = event_piles.insert(std::make_pair(time, e));
    assert(p1.second);
    if (e.should_be_executed()) {
      event_needs(EXECUTION, time);
    }
  }
  void erase_instigating_event(extended_time time) {
    const auto event_pile_iter = event_piles.find(time);
    if (event_pile_iter != event_piles.end()) {
      event_pile_info& pile_info = event_pile_iter->second;
      
      pile_info.instigating_event = nullptr;
      if (pile_info.has_been_executed) {
        event_needs(UNEXECUTION, time);
      }
      else {
        erase_event_impl(time, pile_info, event_pile_iter);
      }
    }
  }
  void erase_event_impl(extended_time time, event_pile_info const& pile_info, decltype(event_piles.find(extended_time())) const& event_pile_iter) {
    if (pile_info.tid) {
      auto& trigger_info = triggers.find(pile_info.tid)->second;
      trigger_info.calls.erase(time);
    }
    //if (was_needing_execution) { event_piles_needing_execution.erase(time); }
    event_piles.erase(event_pile_iter);
  }
  
  // One of the essential, nonintuitive strengths of this system is that
  // execute_event_pile() is a safe operation.
  // If you execute a change that won't actually happen, it will just end
  // up getting invalidated later.
  void execute_event_pile(extended_time time) {
    validate();
    const auto event_pile_iter = event_piles.find(time);
    assert(event_pile_iter != event_piles.end());
    event_pile_info& pile_info = event_pile_iter->second;
    
    assert(next_event_needing(UNEXECUTION) > time);
    
    // If the event already has been carried out in a different way,
    // we need to undo that before proceeding.
    //const bool event_pile_deleted_for_being_out_of_date = unexecute_event_pile(time);
    //if (event_pile_deleted_for_being_out_of_date) return;
    assert (!pile_info.has_been_executed);
    assert (pile_info.should_be_executed());
    
    if (pile_info.tid) {
      auto const& trigger_info = triggers.find(pile_info.tid)->second;
      assert (trigger_info.eventfunc_active_before(time) == pile_info.instigating_event);
      assert (trigger_info.calls.find(time) != trigger_info.calls.end());
    }
    
    if (pile_info.should_be_executed()) {
      // Let's be very explicit about how long the accessor, which is a bit of a hack, is allowed to exist.
      accessor a(this, time, pile_info.tid);
      a.process_event(pile_info.instigating_event.get());
      
      // We might create events (triggers that trigger now, and anticipated_events).
      // Out-of-date versions of those events need to be cleared before we start applying any of the new changes.
      while (!extended_time_manager::is_after_all_calls_triggered_at(next_event_needing(UNEXECUTION), time)) {
        unexecute_event_pile(next_event_needing(UNEXECUTION));
      }
      bool done = false;
      while (!done) {
        // TODO: more efficient?
        done = true;
        for (std::pair<extended_time, std::shared_ptr<const event>> const& ev : a.new_upcoming_events) {
          auto evi = event_piles.find(ev.first);
          if (evi != event_piles.end() && evi->second.needs_unexecution) {
            const bool event_pile_deleted_for_being_out_of_date = unexecute_event_pile(ev.first);
            assert (event_pile_deleted_for_being_out_of_date);
            done = false;
          }
        }
      }
      
      // Order is important: We must update a field's triggers_pointing_at_this record
      //   before calling update_triggers on it.
      pile_info.entity_fields_pile_accessed = std::move(a.entity_fields_preexisting_state_accessed);
      for (entity_field_id const& id : pile_info.entity_fields_pile_accessed) {
        auto& metadata = require_field_metadata_throughout_time(id);
        if (metadata.event_piles_which_accessed_this.empty() || time > *boost::prior(metadata.event_piles_which_accessed_this.end())) {
          metadata.event_piles_which_accessed_this.insert(metadata.event_piles_which_accessed_this.end(), time);
        }
        else {
          const auto p = metadata.event_piles_which_accessed_this.insert(time);
          assert(p.second);
        }
#ifdef TRACKER_A
        tracker_a.track(metadata.event_piles_which_accessed_this);
#endif
        if (pile_info.tid) {
          update_trigger_access_record(id, metadata, time, pile_info.tid, false);
        }
      }
      std::pair<const extended_time, trigger_call_info> const* next_call_ptr = nullptr;
      if (pile_info.tid) {
        cut_off_trigger_events(false, pile_info.tid, time);
        next_call_ptr = next_executed_trigger_call(pile_info.tid, time);
      }
      for (std::pair<extended_time, std::shared_ptr<const event>> const& ev : a.new_upcoming_events) {
        pile_info.anticipated_events.push_back(ev.first);
        event_pile_info e(ev.second);
        if (next_call_ptr) {
          assert (ev.first != next_call_ptr->first); // > vs >= shouldn't matter: anticipated events are never triggers
          if (ev.first > next_call_ptr->first) {
            e.creation_cut_off = true;
          }
        }
        insert_instigating_event(ev.first, e);
      }
      std::sort(pile_info.anticipated_events.begin(), pile_info.anticipated_events.end());
      
      if (pile_info.tid) {
        const auto last_call_ptr = last_executed_trigger_call(pile_info.tid, time);
        if (last_call_ptr) {
          const extended_time last_call_time = last_call_ptr->first;
          event_pile_info const& last_call_pile_info = event_piles.find(last_call_time)->second;
          for (entity_field_id const& accessed_by_last_call : last_call_pile_info.entity_fields_pile_accessed) {
            auto& metadata = get_mut_field_metadata_throughout_time(accessed_by_last_call);
            if (metadata.event_piles_which_accessed_this.find(time) == metadata.event_piles_which_accessed_this.end()) {
              update_trigger_access_record(accessed_by_last_call, metadata, time, pile_info.tid, true);
            }
          }
        }
      }
      pile_info.entity_fields_pile_modified = std::move(a.entity_fields_modified);
      for (entity_field_id const& id : pile_info.entity_fields_pile_modified) {
        auto const& metadata = require_field_metadata_throughout_time(id);
        (*copy_field_change_from_accessor_funcs[id.f.base])(this, id, metadata, time, a.entities.find(id.e)->second.fields);
        update_triggers(false, metadata, time);
      }
      pile_info.triggers_changed.reserve(a.trigger_changes.size());
      for (std::pair<trigger_id, std::shared_ptr<const trigger>> const& t : a.trigger_changes) {
        pile_info.triggers_changed.push_back(t.first);
        impose_trigger_eventfunc(false, triggers[t.first], time, t.second);
        update_trigger(false, t.first, time);
      }
      
      // a is destroyed here
    }
    
    pile_info.has_been_executed = true;
    //event_piles_needing_execution.erase(time);
    validate();
  }
  
  // In essence, this just forgets the consequences of how a
  // particular event happened, without forgetting the fact
  // that it's going to happen.
  // So it too is a "safe" operation (no amount of calling it
  // with bogus values will change us from valid to invalid),
  // because if you undo anything, it's slated to be redone.
  bool unexecute_event_pile(extended_time time) {
    validate();
    const auto event_pile_iter = event_piles.find(time);
    assert (event_pile_iter != event_piles.end());
    event_pile_info& pile_info = event_pile_iter->second;
    
    assert (pile_info.has_been_executed);
    
    // Order is important: For things whose order matters, do the reverse of execute_event_pile.
    for (trigger_id const& tid : pile_info.triggers_changed) {
      update_trigger(true, tid, time);
      impose_trigger_eventfunc(true, triggers.find(tid)->second, time);
    }
    for (entity_field_id const& id : pile_info.entity_fields_pile_modified) {
      auto const& metadata = get_field_metadata_throughout_time(id);
      update_triggers(true, metadata, time);
      (*undo_field_change_funcs[id.f.base])(this, id, metadata, time);
    }
    if (pile_info.tid) {
      const auto last_call_ptr = last_executed_trigger_call(pile_info.tid, time);
      if (last_call_ptr) {
        for (entity_field_id const& accessed_by_last_call : event_piles.find(last_call_ptr->first)->second.entity_fields_pile_accessed) {
          auto& metadata = get_mut_field_metadata_throughout_time(accessed_by_last_call);
          // this 'if' isn't strictly necessary, but I think it provides some optimization
          if (metadata.event_piles_which_accessed_this.find(time) == metadata.event_piles_which_accessed_this.end()) {
            delete_trigger_access_record(accessed_by_last_call, metadata, time, pile_info.tid);
          }
        }
      }
      cut_off_trigger_events(true, pile_info.tid, time);
    }
    for (extended_time t : pile_info.anticipated_events) {
      erase_instigating_event(t);
    }
    pile_info.anticipated_events.clear();
    for (entity_field_id const& id : pile_info.entity_fields_pile_accessed) {
      auto& metadata = get_mut_field_metadata_throughout_time(id);
      const auto j = metadata.event_piles_which_accessed_this.erase(time);
#ifdef TRACKER_A
      tracker_a.track(metadata.event_piles_which_accessed_this);
#endif
      assert(j);
      if (pile_info.tid) {
        delete_trigger_access_record(id, metadata, time, pile_info.tid);
      }
    }
    pile_info.entity_fields_pile_accessed.clear();
    pile_info.entity_fields_pile_modified.clear();
    pile_info.triggers_changed.clear();
    
    pile_info.has_been_executed = false;
    pile_info.needs_unexecution = false;
    //event_piles_needing_unexecution.erase(time);
    if (pile_info.should_be_executed()) {
      event_needs(EXECUTION, time);
    }
    if (!pile_info.instigating_event) {
      erase_event_impl(time, pile_info, event_pile_iter);
      validate();
      return true;
    }
    validate();
    return false;
  }
};

} //end namespace time_steward_system

#endif
