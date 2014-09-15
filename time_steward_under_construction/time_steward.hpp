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
#include <memory>

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
 */


// hackly copied from /usr/include/c++/4.9.0/bits/unique_ptr.h
// so we don't need c++1y libs just c++11
namespace futurestd {
  using namespace std;
  template<typename _Tp>
    struct _MakeUniq
    { typedef unique_ptr<_Tp> __single_object; };

  template<typename _Tp>
    struct _MakeUniq<_Tp[]>
    { typedef unique_ptr<_Tp[]> __array; };

  template<typename _Tp, size_t _Bound>
    struct _MakeUniq<_Tp[_Bound]>
    { struct __invalid_type { }; };

  /// std::make_unique for single objects
  template<typename _Tp, typename... _Args>
    inline typename _MakeUniq<_Tp>::__single_object
    make_unique(_Args&&... __args)
    { return unique_ptr<_Tp>(new _Tp(std::forward<_Args>(__args)...)); }

  /// std::make_unique for arrays of unknown bound
  template<typename _Tp>
    inline typename _MakeUniq<_Tp>::__array
    make_unique(size_t __num)
    { return unique_ptr<_Tp>(new typename remove_extent<_Tp>::type[__num]()); }

  /// Disable std::make_unique for arrays of known bound
  template<typename _Tp, typename... _Args>
    inline typename _MakeUniq<_Tp>::__invalid_type
    make_unique(_Args&&...) = delete;
}
using futurestd::make_unique;



uint64_t siphash24(const void *src,
                   unsigned long src_sz,
                   const char key[16]);

namespace time_steward_system {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
// 128 bits of random stuff (presumed unique to this library)
const char siphash_key[16] = { 0xb7, 0xac, 0x3d, 0xf8, 0xc3, 0xa2, 0x8c, 0xd9, 0x3a, 0x10, 0x91, 0x68, 0x09, 0x02, 0x74, 0x0e };
#pragma GCC diagnostic pop


// siphash_id takes anything that's hashable and turns it into a
// fixed-size, statistically almost certainly unique, equality-comparable
// and orderable data structure (i.e. 128 bits of data).  The ==
// is true iff the things are the same; the < is an arbitrary total ordering.
class siphash_id {
  typedef std::array<uint64_t,2> data_type;
public:
  // Implementation note:
  // A reply from the SipHash developers about our idea to get a
  // large enough result from SipHash to be statistically unique
  // by hashing each of N and N+1 and keeping both results:
  //
  // Hi Eli,
  //
  // the development of a 128-bit SipHash is under discussion, see e.g.
  // this Twitter thread:
  //https://twitter.com/Kryptoblog/status/397285808714960897
  //
  // The trick you use seems to work, but it'd perhaps be more elegant to
  // just hash the same info under two distinct keys, or (say) under key K
  // and key (K xor 0xffff..ffff). That's still twice as slow as a single
  // pass though.
  //
  // What you should not do obviously is "hash the hash" to get another 64
  // bits, as any collision would propagate.
  //
  // When/if a SipHash-128 is developed, we'll advertize it on Twitter.
  //
  // Best,
  //
  // JP
  
  // These constructors combine all the entropy from all the arguments.
  siphash_id(uint64_t i){
    uint64_t in[] = { i };
    data_[0] = siphash24((void*)in, sizeof(in), siphash_key);
    ++in[0];
    data_[1] = siphash24((void*)in, sizeof(in), siphash_key);
  }
  siphash_id(siphash_id a, uint64_t i){
    uint64_t in[] = { a.data_[0], a.data_[1], i };
    data_[0] = siphash24((void*)in, sizeof(in), siphash_key);
    ++in[0];
    data_[1] = siphash24((void*)in, sizeof(in), siphash_key);
  }
  siphash_id(siphash_id a, siphash_id b){
    uint64_t in[] = { a.data_[0], a.data_[1], b.data_[0], b.data_[1] };
    data_[0] = siphash24((void*)in, sizeof(in), siphash_key);
    ++in[0];
    data_[1] = siphash24((void*)in, sizeof(in), siphash_key);
  }
  // null siphash
  constexpr siphash_id() : data_({{ 0, 0 }}) {}
  constexpr static siphash_id null() {
    return siphash_id();
  }
  // least and greatest nonnull siphash
  constexpr static siphash_id least() {
    return siphash_id(data_type{{ 1, 0 }});
  }
  constexpr static siphash_id greatest() {
    return siphash_id(data_type{{ uint64_t(0)-1, uint64_t(0)-1 }});
  }

  bool operator<(siphash_id const& o)const {
    if (data_[1] < o.data_[1]) return true;
    if (data_[1] > o.data_[1]) return false;
    return data_[0] < o.data_[0];
  }
  bool operator==(siphash_id const& o)const {
    return (data_[0] == o.data_[0]) && (data_[1] == o.data_[1]);
  }
  bool operator!=(siphash_id const& o)const {
    return (data_[0] != o.data_[0]) || (data_[1] != o.data_[1]);
  }
private:
  constexpr siphash_id(data_type data):data_(data){}
  data_type data_;
  friend class std::hash<siphash_id>;
};

} //end namespace time_steward_system
namespace std {
  template<>
  class hash<time_steward_system::siphash_id> {
    public:
    size_t operator()(time_steward_system::siphash_id const& i)const {
      // Since the siphash is already a hash, just return some of it.
      return i.data_[0];
    }
  };
}
namespace time_steward_system {

template<typename IntType>
struct combine_hash_with_integer {
  siphash_id operator()(siphash_id const& hash, IntType i)const noexcept {
    return siphash_id(hash, static_cast<uint64_t>(i));
  }
};

typedef siphash_id time_id;

template<typename TimeType = int64_t, TimeType Never = std::numeric_limits<TimeType>::min(), TimeType MinTime = Never+1, class CombineHashWithTimeTypeFuncType = combine_hash_with_integer<TimeType>>
struct time_type_info {
  static_assert(Never < MinTime, "Never must be less than MinTime");
  typedef TimeType time_type;
  static const TimeType never = Never;
  static const TimeType min_time = MinTime;
  typedef CombineHashWithTimeTypeFuncType combine_hash_with_time_type_func_type;
};


template<typename EntitySubclass> struct entity_id;
namespace impl {
template<typename T> entity_id<T> create_entity_id(siphash_id id);
template<typename T> siphash_id get_untyped_id(entity_id<T> id);
}

template<typename EntitySubclass>
struct entity_id {
  entity_id() {}

  // entity_id<E1> can be converted to entity_id<E2> iff E1* can be converted to E2*
  template<typename DerivedEntity> entity_id(entity_id<DerivedEntity> id, EntitySubclass* = static_cast<DerivedEntity*>(nullptr))
    : id_(id.id_) {}
  
  template<class T, class U>
  friend inline entity_id<T> reinterpret_entity_id(entity_id<U> const& r);

  explicit operator bool()const {
    return id_ != siphash_id::null();
  }
private:
  siphash_id id_;

  template<typename T> friend struct entity_id;
  template<typename T> friend entity_id<T> impl::create_entity_id(siphash_id id);
  template<typename T> friend siphash_id impl::get_untyped_id(entity_id<T> id);
};
template<class T, class U>
inline entity_id<T> reinterpret_entity_id(entity_id<U> const& r) {
  return impl::create_entity_id<T>(r.id_);;
}
template<class T, class U>
bool operator==(entity_id<T> a, entity_id<U> b) { return impl::get_untyped_id(a) == impl::get_untyped_id(b); }
template<class T, class U>
bool operator!=(entity_id<T> a, entity_id<U> b) { return impl::get_untyped_id(a) != impl::get_untyped_id(b); }

namespace impl {
template<typename T>
entity_id<T> create_entity_id(siphash_id id) {
  entity_id<T> e;
  e.id_ = id;
  return e;
}
template<typename T>
siphash_id get_untyped_id(entity_id<T> id) {
  return id.id_;
}
}
template<typename T>
entity_id<T> global_object_id() { return impl::create_entity_id<T>(siphash_id::least()); }


template<typename TimeSteward>
class time_steward_accessor;

template<typename EntitySubclass>
class entity_ref {
public:
  // implicit conversion from non-const to const and derived to base
  template<typename DerivedEntity> entity_ref(entity_ref<DerivedEntity> er)
    : e_(er.e_), id_(er.id_) {}

  entity_id<EntitySubclass> id()const {
    return id_;
  }
  EntitySubclass* operator->()const {
    return e_;
  }
  EntitySubclass& operator*()const {
    return *e_;
  }
  explicit operator bool()const {
    return e_;
  }
  
  friend bool operator==(entity_ref a, entity_ref b) { return a.e_ == b.e_; }
  friend bool operator!=(entity_ref a, entity_ref b) { return a.e_ != b.e_; }

private:
  template<typename> friend class time_steward_accessor;
  template<typename> friend class entity_ref;
  template<class T, class U>
  friend entity_ref<T> dynamic_pointer_cast(entity_ref<U> const& r);

  entity_ref(EntitySubclass* e, entity_id<EntitySubclass> id) : e_(e), id_(id) {}

  EntitySubclass* e_;
  entity_id<EntitySubclass> id_;
};
template<class T, class U>
inline entity_ref<T> dynamic_pointer_cast(entity_ref<U> const& r) {
  return entity_ref<T>(dynamic_cast<T*>(r.e_), reinterpret_entity_id<T>(r.id()));
}

} //end namespace time_steward_system
namespace std {
  template<typename EntitySubclass>
  class hash<time_steward_system::entity_id<EntitySubclass>> {
    public:
    size_t operator()(time_steward_system::entity_id<EntitySubclass> const& i)const {
      return std::hash<time_steward_system::siphash_id>()(time_steward_system::impl::get_untyped_id(i));
    }
  };
}
namespace time_steward_system {

template<typename EntitySubclass>
using entity_const_ref = entity_ref<const EntitySubclass>;

class standard_entity_base {
public:
  virtual standard_entity_base* clone()const = 0;
  virtual ~standard_entity_base(){}
};
class null_entity : public standard_entity_base {
public:
  null_entity* clone()const override{ return new null_entity(*this); }
};

template<typename TimeSteward>
class time_steward_accessor {
  typedef typename TimeSteward::entity entity;
  typedef typename TimeSteward::extended_time extended_time;
  typedef typename TimeSteward::time_type time_type;
  typedef typename TimeSteward::action action;
  typedef typename TimeSteward::consequential_event consequential_event;
  typedef typename TimeSteward::trigger trigger;
  typedef typename TimeSteward::trigger_id trigger_id;
  typedef uint32_t invalidation_counter_t;
  
  struct trigger_info {
    trigger_info(std::shared_ptr<const trigger> t, bool in_queue):t(t),in_queue(in_queue){}
    std::shared_ptr<const trigger> t;
    bool in_queue;
  };
  
  struct field_metadata {
    persistent_map<trigger_id, trigger_info> triggers;
    bool accessed_preexisting_state;
    bool ever_modified;
    bool all_triggers_queued;
    void accessed(time_steward_accessor const& acc) {
      if (acc.event_whenfunc_access_tracker) { acc.event_whenfunc_access_tracker->insert(fid); }
      if (!ever_modified) { accessed_preexisting_state = true; }
    }
    void modified(time_steward_accessor& acc, field_id fid) {
      ever_modified = true;
      if (!all_triggers_queued) {
        for (std::pair<trigger_id, trigger_info> const& i : triggers) {
          if (!i.in_queue) {
            acc.triggers_queue.emplace(fid, i.first);
            i.in_queue = true;
          }
        }
        all_triggers_queued = true;
      }
    }
  };
    
  template<typename FieldsList>
  struct entity_info {
    entity_id id;
    FieldsList fields;
    std::array<field_metadata, FieldsList::size> metadata;
  };
  
public:  
  template<typename FieldContents, typename FieldsList>
  inline FieldContents const& get(entity_info<FieldsList> const& e)const {
    const field_id fid(e.id, FieldsList::index_of<FieldContents>::idx);
    if (event_whenfunc_access_tracker) { event_whenfunc_access_tracker->insert(fid); }
    metadata[idx].accessed(*this);
    return e.fields.get<FieldContents>();
  }
  template<typename FieldContents, typename FieldsList>
  inline FieldContents& get_mut(entity_info<FieldsList>& e) {
    const size_t idx = FieldsList::index_of<FieldContents>::idx;
    const field_id fid(e.id, idx);
    metadata[idx].accessed(*this);
    metadata[idx].modified(*this, fid);
    return e.fields.get_mut<FieldContents>();
  }
  template<typename FieldContents, typename FieldsList>
  inline FieldContents& replace_field(entity_info<FieldsList>& e, FieldContents new_contents) {
    const size_t idx = FieldsList::index_of<FieldContents>::idx;
    const field_id fid(e.id, idx);
    metadata[idx].modified(*this, fid);
    return e.fields.replace<FieldContents>(new_contents);
  }
private:
    
    
  struct queued_trigger_info {
    std::shared_ptr<const trigger> t;
    entity_id<entity> eid;
    trigger_id tid;
    invalidation_counter_t invalidation_counter;
  };
  struct event_info {
    event const* e;
  };
  struct upcoming_event_info {
    std::shared_ptr<const event> e;
    std::unordered_map<entity_id<entity>, invalidation_counter_t> dependencies;
  };
  struct transient_entity_info {
    transient_entity_info(std::unique_ptr<entity>&& e, triggers, bool accessed_preexisting_state, bool modified)
      : e(std::move(e)), triggers(std::move(triggers)), accessed_preexisting_state(accessed_preexisting_state), modified(modified) {}
    std::unique_ptr<entity> e;
    std::map<trigger_id, trigger_info> triggers;
    uint32_t invalidation_counter;
    bool accessed_preexisting_state;
    bool modified;
  };
  public:
    // If we already overwrote (or created) an entity,
    // then this is not accessing the entity's state *before* this time,
    // so it doesn't count as an access.
    // TODO maybe allow non dynamic castable (nonvirtual) entities
    // TODO do we need both get()s?
    template<typename T>
    entity_ref<T const> get(entity_id<T> id)const {
      if (event_whenfunc_access_tracker) { event_whenfunc_access_tracker->insert(id); }
      return dynamic_pointer_cast<T const>(
        entity_ref<entity const>(require_local_entity(reinterpret_entity_id<entity>(id), false).e.get(), id));
    };
    template<typename T>
    entity_ref<T const> get(entity_id<T const> id)const {
      if (event_whenfunc_access_tracker) { event_whenfunc_access_tracker->insert(id); }
      return dynamic_pointer_cast<T const>(
        entity_ref<entity const>(require_local_entity(reinterpret_entity_id<entity>(id), false).e.get(), id));
    };
    template<typename T>
    entity_ref<T> get_mut(entity_id<T> id) {
      return dynamic_pointer_cast<T>(
        entity_ref<entity>(require_local_entity(id, true).e.get(), id));
    };
    void create_event(std::unique_ptr<consequential_event> e) {
      fresh_new_upcoming_events.push_back(std::move(e));
    };
    void create_trigger(entity_id<entity> eid, trigger_id tid, std::shared_ptr<const trigger> t) {
      transient_entity_info& info = require_local_entity(eid, false);
      info.triggers.insert(std::pair<trigger_id, trigger_info>(tid, trigger_info(t, true)));
      triggers_queue.emplace(t, eid, tid, info.invalidation_counter);
    };
    void delete_trigger(entity_id<entity> eid, trigger_id tid) {
      require_local_entity(eid, false).triggers.erase(tid, t);
    };
    time_type now()const {
      return time_.base_time;
    }
    template<typename T>
    entity_ref<T> create_entity(std::unique_ptr<T> e) {
      // All entity IDs must be unique.
      // This function is one of the two ways that new entity IDs are created.
      // (The other is the fixed entity ID of the global object.)
      // As long as the other requirement is preserved - that
      // no two events are allowed to happen at the same extended_time -
      // this will result in all unique entity IDs.
      T* entity_ptr = &*e;
      ++entities_created_;
      const entity_id<T> id = impl::create_entity_id<T>(siphash_id(time_.id, entities_created_));
      const auto inserted = local_entities_.insert(
        std::pair<entity_id<entity>, transient_entity_info>(
          id,
          transient_entity_info(std::move(e), nothing, false, true)));
      assert(inserted.second);
      return entity_ref<T>(entity_ptr, id);
    }
    // replaces the entity with id "id" with a new entity value
    template<typename T>
    entity_ref<T> replace_entity(entity_id<entity> id, std::unique_ptr<T> e) {
      T* entity_ptr = &*e;
      // insert a dummy unique_ptr first so that we don't lose the
      // ownership if insertion hits an object that's already there.
      const auto inserted = local_entities_.insert(
        std::pair<entity_id<entity>, transient_entity_info>(
          id,
          transient_entity_info(std::unique_ptr<entity>(), nothing, false, true)));
      inserted.first->second.e = std::move(e);
      // (retain old value of accessed_preexisting_state if it already existed)
      inserted.first->second.modified = true;
      return entity_ref<T>(entity_ptr, reinterpret_entity_id<T>(id));
    }
    void delete_entity(entity_id<entity> id) {
      replace_entity(id, std::unique_ptr<null_entity>(new null_entity()));
    }
    
    time_steward_accessor(time_steward_accessor const&) = delete;
    time_steward_accessor(time_steward_accessor&&) = delete;
    time_steward_accessor& operator=(time_steward_accessor const&) = delete;
    time_steward_accessor& operator=(time_steward_accessor&&) = delete;
  private:
    TimeSteward const* ts_;
    extended_time time_;
    size_t entities_created_;
    mutable std::unordered_map<entity_id<entity>, transient_entity_info> local_entities_;
    std::queue<queued_event_info> events_queue;
    std::queue<upcoming_event_info> triggers_queue;
    std::vector<std::unique_ptr<consequential_event>> fresh_new_upcoming_events;
    std::vector<std::pair<time_type, upcoming_event_info>> stale_new_upcoming_events;
    std::unordered_set<entity_id<entity>> fresh_modifications;
    std::unordered_set<entity_id<entity>> *event_whenfunc_access_tracker;

    transient_entity_info& require_local_entity(entity_id<entity> id, bool modified)const {
      const auto iter = local_entities_.find(id);
      if (iter == local_entities_.end()) {
        // accessed_preexisting_state should be true iff we do ts_->get_provisional_entity_data_before for an entity.
        std::unique_ptr<entity> e(ts_->get_provisional_entity_data_before(id, time_)->clone());
        std::map<trigger_id, std::shared_ptr<trigger>> triggers = ts_->get_provisional_triggers_active_before(id, time_);
        const auto inserted = local_entities_.insert(
          std::pair<entity_id<entity>, transient_entity_info>(
            id,
            transient_entity_info(std::move(e), triggers, true, modified)));
        assert(inserted.second);
        iter = inserted.first;
      }
      
      if (modified) {
        iter->second.modified = true;
        fresh_modifications.insert(id);
      }
      
      return iter->second;
    }
    
    bool upcoming_event_info_still_valid(upcoming_event_info const& i)const {
      for (std::pair<entity_id<entity>, invalidation_counter_t> p : i.dependencies) {
        transient_entity_info const& info = require_local_entity(p.first, false);
        if (info.invalidation_counter != p.second) { return false; }
      }
      return true;
    }
    
    time_steward_accessor(TimeSteward const* ts, extended_time const& time):ts_(ts),time_(time),entities_created_(0){}
    void process_follow_ups() {
      for (entity_id<entity> id : fresh_modifications) {
        transient_entity_info& info = require_local_entity(id, false);
        ++info.invalidation_counter;
        for (std::pair<trigger_id, trigger_info> const& i : info.triggers) {
          if (!i.in_queue) {
            triggers_queue.emplace(i.second.t, id, i.first, info.invalidation_counter);
            i.in_queue = true;
          }
        }
      }
      fresh_modifications.clear();
      
      while (!triggers_queue.empty()) {
        const queued_trigger_info t = triggers_queue.front();
        triggers_queue.pop();
        transient_entity_info& info = require_local_entity(t.eid, false);
        if (info.invalidation_counter == t.invalidation_counter) {
          (*t.t)(this, entity_ref<entity>(info.e.get(), t.eid));
          process_follow_ups();
          return;
        }
      }
      
      for (auto new_upcoming_event : fresh_new_upcoming_events) {
        std::unordered_set<entity_id<entity>> tracker;
        event_whenfunc_access_tracker = &tracker;
        const time_type t = new_upcoming_event.func->when(this, new_upcoming_event.eid);
        event_whenfunc_access_tracker = nullptr;
        upcoming_event_info inf(new_upcoming_event.func);
        for (entity_id<entity> id : tracker) {
          inf.dependencies.insert(std::make_pair(id, require_local_entity(id, false).invalidation_counter));
        }
        
        if (t != never) {
          if (t == time_.base_time) {
            events_queue.push(std::move(inf));
          }
          else {
            assert(t > time_.base_time); // TODO an exception
            stale_new_upcoming_events.push_back(std::pair<time_type, upcoming_event_info>(t, std::move(inf)));
          }
        }
      }
      fresh_new_upcoming_events.clear();
      
      while (!events_queue.empty()) {
        const upcoming_event_info e = events_queue.front();
        events_queue.pop();
        if (upcoming_event_info_still_valid(e)) {
          (*e.e)(this);
          process_follow_ups();
          return;
        }
      }
    }
    friend TimeSteward;
};

namespace impl {
template<typename TimeTypeInfo>
struct extended_time {
  typedef typename TimeTypeInfo::time_type time_type;

    time_type base_time;
    time_id id;
    struct first_t{}; struct last_t{}; static const first_t first; static const last_t last;
    extended_time(time_type base_time, first_t):base_time(base_time),id(siphash_id::least()){}
    extended_time(time_type base_time,  last_t):base_time(base_time),id(siphash_id::greatest()){}
    extended_time(time_type base_time, siphash_id id):base_time(base_time),id(id){}
    
    bool operator==(extended_time const& o)const { return id == o.id; }
    bool operator!=(extended_time const& o)const { return id != o.id; }
    bool operator<(extended_time const& o)const {
      if (base_time < o.base_time) return true;
      if (base_time > o.base_time) return false;
      return id < o.id;
    }
    bool operator>(extended_time const& o)const { return o < *this; }
    bool operator>=(extended_time const& o)const { return !(*this < o); }
    bool operator<=(extended_time const& o)const { return !(o < *this); }

  // For use with std::unordered_[set|map].
  // Since the id is already a hash, just return some of it.
  struct hash {
    size_t operator()(extended_time const& t)const { return std::hash<siphash_id>()(t.id); }
  };
};
}

template<class TimeTypeInfo = time_type_info<>, class EntityData = standard_entity_base/*, class Event = */>
class time_steward {
public:
  typedef time_steward_accessor<time_steward> accessor;
  typedef typename TimeTypeInfo::time_type time_type;
  static const time_type min_time = TimeTypeInfo::min_time;
  static const time_type never = TimeTypeInfo::never;
  typedef EntityData entity;
  // TODO can the action/event/trigger stuff be template parameters rather than always being virtual classes to be subclassed?
  class action {
  public:
    virtual void operator()(accessor* accessor)const = 0;
    virtual ~action(){}
  };
  typedef action event;
  typedef action trigger;
  class consequential_event : virtual public event {
  public:
    virtual void operator()(accessor* accessor)const = 0;
    virtual time_type when(accessor const* accessor)const = 0;
  };
private:
  typedef typename TimeTypeInfo::combine_hash_with_time_type_func_type combine_hash_with_time_type_func_type;
  typedef impl::extended_time<TimeTypeInfo> extended_time;
  typedef std::set<extended_time> time_set;
  struct entity_change {
    std::unique_ptr<entity> e;
    time_set anticipated_instigating_events;
  };
  typedef std::map<extended_time, entity_change> entity_changes_map;
  typedef typename extended_time::hash extended_time_hash;
  typedef std::unordered_set<entity_id<entity>> entity_set;
  struct event_pile_info {
    event_with_dependencies(std::unique_ptr<event>&& instigating_event) :
      instigating_event (instigating_event),
      num_instigating_event_creation_dependencies_cut_off (0),
      has_been_executed (false)
      {}
    extended_time instigating_event_creation_time;
    entity_set instigating_event_creation_dependencies;
    uint32_t num_instigating_event_creation_dependencies_cut_off;
    std::unique_ptr<event> instigating_event;
    entity_set entities_pile_accessed;
    entity_set entities_pile_modified;
    time_set instigating_events_pile_created;
    bool has_been_executed;
    bool should_be_executed()const { return instigating_event && (num_instigating_event_creation_dependencies_cut_off == 0); }
  };
  
  struct field_metadata {
    std::map<extended_time, persistent_map<trigger_id, trigger_info>> triggers_changes;
    std::unordered_set<event_pile_id> event_piles_which_accessed_this;
    std::unordered_set<event_pile_id> event_piles_whose_instigating_event_creation_accessed_this;
  };
    
  template<class FieldsList>
  struct entity_throughout_existence_info {
    field_maps<FieldsList> fields;
    std::array<field_metadata, FieldsList::size> metadata;
  }
  struct entity_throughout_time_info {
    std::map<extended_time, entity_throughout_existence_info> changes;
  };
  
  // The events map doesn't need to be ordered (even though it has a meaningful order)
  // because it's just for looking up events whose times we know.
  typedef std::unordered_map<extended_time, event_pile_info, extended_time_hash> event_piles_map;
  typedef std::unordered_map<entity_id<entity>, entity_throughout_time_info> entities_map;

  friend class time_steward_accessor<time_steward>;
public:
  
  time_steward(std::unique_ptr<entity> initial_state_of_global_object) {
    entities[global_object_id<entity>()].changes.insert(std::make_pair(extended_time(TimeTypeInfo::min_time, extended_time::first), std::move(initial_state_of_global_object)));
  }
  
  void insert_fiat_event(time_type time, distinguisher, std::unique_ptr<event> e) {
    // This function is one of exactly two places where
    // (persistent) extended_times are created.
    // Uniqueness justification:
    // TODO
    const extended_time t(time, combine_hash_with_time_type_func_type()(siphash_id(distinguisher), base_time));
    // TODO throw an exception if the user inserts two events at the same time with the same distinguisher
    insert_instigating_event(t, std::move(e));
  }
  void erase_fiat_event(time_type time, uint64_t distinguisher) {
    const extended_time t(time, siphash_id(distinguisher));
    erase_instigating_event(t);
  }
  
  // For external client code to examine the state - 
  //   e.g., for display.
  // Output of these should never find their way into internal client code,
  //   and it's a little strange (but ok) to have them affect FiatEvents.
  template<typename T>
  entity_ref<T const> get_entity_data_after(entity_id<T> const& id, time_type const& time) {
    // Note: For collision safety, these extended_times must not persist.
    return dynamic_pointer_cast<T const>(entity_ref<entity const>(
      get_actual_entity_data_before(id, extended_time(time, extended_time::last))));
  }
  std::unique_ptr<accessor> accessor_after(time_type const& time) {
    extended_time et(time, extended_time::last);
    update_through_time(et);
    return std::unique_ptr<accessor>(new accessor(this, et));
  }
  
  // Some functions for external client code to regulate how much processing the time_steward
  //   does in advance of when it's needed.
  void update_until_time(time_type const& time) {
    // Note: For collision safety, these extended_times must not persist.
    update_until_time(extended_time(time, extended_time::first));
  }
  void update_through_time(time_type const& time) {
    // Note: For collision safety, these extended_times must not persist.
    update_through_time(extended_time(time, extended_time::last));
  }
private:
  entities_map entities;
  event_piles_map event_piles;
  time_set event_piles_not_correctly_executed;
  
  void update_through_time(extended_time const& time) {
    while (!is_updated_through(time)) execute_event_pile(*event_piles_not_correctly_executed.begin());
  }
  void update_until_time  (extended_time const& time) {
    while (!is_updated_until  (time)) execute_event_pile(*event_piles_not_correctly_executed.begin());
  }
  bool is_updated_until  (extended_time const& time)const {
    return event_piles_not_correctly_executed.empty() || *event_piles_not_correctly_executed.begin() >= time;
  }
  bool is_updated_through(extended_time const& time)const {
    return event_piles_not_correctly_executed.empty() || *event_piles_not_correctly_executed.begin() >  time;
  }
  entity const* get_actual_entity_data_before(entity_id<entity> const& id, extended_time const& time) {
    update_until_time(time);
    return get_provisional_entity_data_before(id, time);
  }
  entity const* get_provisional_entity_data_before(entity_id<entity> const& id, extended_time const& time)const {
    const auto entity_stream_iter = entities.find(id);
    if (entity_stream_iter == entities.end()) {
      assert(false); // TODO: This might be an external client code error, so throw an exception instead.
    }
    
    entity_throughout_time_info const& e = entity_stream_iter->second;
    const auto next_change_iter = e.changes.upper_bound(time);
    if (next_change_iter == e.changes.begin()) {
      assert(false); // TODO: This might be an external client code error, so throw an exception instead.
    }
    return &*boost::prior(next_change_iter)->second;
  }
  
  void insert_instigating_event(extended_time const& time, std::shared_ptr<event> e) {
    const auto p1 = event_piles.insert(std::make_pair(time, event_pile_info(std::move(e))));
    assert(p1.second);
    const auto p2 = event_piles_not_correctly_executed.insert(time);
    assert(p2.second);
  }
  void erase_instigating_event(extended_time const& time) {
    const auto event_pile_iter = event_piles.find(time);
    if (event_pile_iter != event_piles.end()) {
      event_piles_not_correctly_executed.erase(time);
      event_pile_info& pile_info = event_pile_iter->second;
      pile_info.instigating_event = nullptr;
      for (entity_id id : pile_info.instigating_event_creation_dependencies) {
        const auto entity_stream_iter = entities.find(id);
        assert (entity_stream_iter != entities.end());
        entity_throughout_time_info& e = entity_stream_iter->second;
        boost::prior(e.changes.upper_bound(pile_info.instigating_event_creation_time))->anticipated_instigating_events.erase(time);
      }
      if (pile_info.has_been_executed) {
        event_piles_not_correctly_executed.insert(time);
      }
      else {
        event_piles_not_correctly_executed.erase(time);
        event_piles.erase(event_iter);
      }
    }
  }
  // One of the essential, nonintuitive strengths of this system is that
  // execute_event_pile() is a safe operation.
  // If you execute a change that won't actually happen, it will just end
  // up getting invalidated later.
  void execute_event_pile(extended_time const& time) {
    const auto event_pile_iter = event_piles.find(time);
    assert(event_pile_iter != event_piles.end());
    event_pile_info& pile_info = event_pile_iter->second;
    
    // If the event already has been carried out in a different way,
    // we need to undo that before proceeding.
    const bool event_pile_deleted_for_being_out_of_date = unexecute_event_pile(time);
    if (event_pile_deleted_for_being_out_of_date) return;
    
    if (pile_info.should_be_executed()) {
      // Let's be very explicit about how long the accessor, which is a bit of a hack, is allowed to exist.
      accessor a(this, time);
      (*pile_info.instigating_event)(&a);
      a.process_follow_ups();
      
      for (std::pair<const entity_id<entity>, typename accessor::entity_and_ways_it_has_been_referenced>& i : a.local_entities_) {
        entity_throughout_time_info& e = entities[i.first]; // Default-construct if it's not there
        //entity_changes_map::iterator current_change;
        if (i.second.accessed_preexisting_state) {
          event.accessed_entities.insert(i.first);
          const auto p = entities[i.first].event_piles_which_accessed_this.insert(time);
          assert(p.second);
        }
        if (i.second.modified) {
          event.modified_entities.insert(i.first);
          const auto p = e.changes.insert(std::make_pair(time, std::move(i.second.e)));
          assert(p.second);
          //current_change = p.first;
          if (p.first != e.changes.begin()) {
            const auto cut_off = boost::prior(p.first);
            for (auto ev = cut_off->second.anticipated_instigating_events.upper_bound(time); ev != cut_off->second.anticipated_instigating_events.end(); ++ev) {
              event_pile_info& pile_info2 = *event_piles.find(*ev);
              if (pile_info2.num_instigating_event_creation_dependencies_cut_off == 0) {
                if (pile_info2.has_been_executed) { event_piles_not_correctly_executed.insert(*ev); }
                else                              { event_piles_not_correctly_executed.erase (*ev); }
              }
              ++pile_info2.num_instigating_event_creation_dependencies_cut_off;
            }
          }
        }
        //else {
        //  current_change = boost::prior(e.changes.upper_bound(time));
        //}
      }
      uint64_t distinguisher = 0;
      for (accessor::upcoming_event_info ev : a.stale_new_upcoming_events) {
        if (a.upcoming_event_info_still_valid(ev)) {
          // This function is one of exactly two places where
          // (persistent) extended_times are created.
          // Uniqueness justification:
          // TODO
          const extended_time t(ev.when, siphash_id(time.id, distinguisher));
          insert_instigating_event(t, ev.e);
          for (std::pair<entity_id<entity>, invalidation_counter_t> p : ev.dependencies) {
            const auto i = entities.find(p.first); assert(i != entities.end());
            const auto j = i->changes.upper_bound(time); assert(j != i->changes.begin());
            boost::prior(j)->anticipated_instigating_events.insert(t);
          }
        }
      }
      // a is destroyed here
    }
    
    pile_info.has_been_executed = true;
    event_piles_not_correctly_executed.erase(time);
  }
  
  // In essence, this just forgets the consequences of how a
  // particular event happened, without forgetting the fact
  // that it's going to happen.
  // So it too is a "safe" operation (no amount of calling it
  // with bogus values will change us from valid to invalid),
  // because if you undo anything, it's slated to be redone.
  bool unexecute_event_pile(extended_time const& time) {
    const auto event_pile_iter = event_piles.find(time);
    assert (event_pile_iter != event_piles.end());
    event_pile_info& pile_info = event_pile_iter->second;
    if (!pile_info.has_been_executed) return;
    
    for (extended_time t : pile_info.instigating_events_pile_created) {
      erase_instigating_event(t);
    }
    for (entity_id<entity> const& id : pile_info.entities_pile_accessed) {
      const auto i = entities.find(id); assert(i != entities.end());
      i->second.event_piles_which_accessed_this.erase(time);
    }
    for (entity_id<entity> const& id : pile_info.entities_pile_modified) {
      erase_entity_change(e, time);
      const auto entity_stream_iter = entities.find(id);
      assert (entity_stream_iter != entities.end());
      entity_throughout_time_info& e = entity_stream_iter->second;
      invalidate_event_piles_that_accessed_entity_when_it_was_defined_by_the_change_at(e, time);
      
      const auto entity_change_iter = e.changes.find(time);
      assert (entity_change_iter != e.changes.end());
      
      const time_set removed_instigating_events = std::move(entity_change_iter->anticipated_instigating_events); entity_change_iter->anticipated_instigating_events = time_set();
      for (extended_time t : removed_instigating_events) {
        erase_instigating_event(t);
      }
      if (entity_change_iter != e.changes.begin()) {
        const auto cut_off = boost::prior(entity_change_iter);
        for (auto ev = cut_off->second.anticipated_instigating_events.upper_bound(time); ev != cut_off->second.anticipated_instigating_events.end(); ++ev) {
          event_pile_info& pile_info2 = event_piles.find(*ev);
          --pile_info2.num_instigating_event_creation_dependencies_cut_off;
          if (pile_info2.should_be_executed()) {
            event_piles_not_correctly_executed.insert(*ev);
          }
        }
      }
      
      e.changes.erase(entity_change_iter);
      if (e.changes.empty()) {
        assert (e.event_piles_which_accessed_this.empty());
        entities.erase(entity_stream_iter);
      }
    }
    pile_info.entities_pile_accessed.clear();
    pile_info.entities_pile_modified.clear();
    pile_info.instigating_events_pile_created.clear();
    
    pile_info.has_been_executed = false;
    if (pile_info.should_be_executed()) {
      event_piles_not_correctly_executed.insert(time);
    }
    if (!pile_info.instigating_event) {
      event_piles.erase(event_iter);
      return true;
    }
    return false;
  }
  
  void invalidate_event_piles_that_accessed_entity_when_it_was_defined_by_the_change_at(entity_throughout_time_info& e, extended_time const& change_time) {
    const auto next_change = e.changes.upper_bound(change_time);
    // upper_bound(), because references at the same extended_time as a event_pile
    // are that event_pile's own references, and refer to the previous state of the entity.
    const auto invalidated_event_piles_begin = e.event_piles_which_accessed_this.upper_bound(change_time);
    const auto invalidated_event_piles_end = (next_change == e.changes.end()) ? e.event_piles_which_accessed_this.end() : e.event_piles_which_accessed_this.upper_bound(next_change->first);
    // TODO: better to do this with something like std::move?
    for (auto i = invalidated_event_piles_begin; i != invalidated_event_piles_end; ) {
      assert(*i != change_time);
      event_piles_not_correctly_executed.insert(*i);
      e.event_piles_which_accessed_this.erase(i++);
    }
  }
};

} //end namespace time_steward_system

