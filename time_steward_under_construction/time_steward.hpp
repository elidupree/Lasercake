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

// TODO: move all the data structures stuff before namespace time_steward_system to a better file
template<typename ...T> using persistent_map = std::map<T...>;
template<typename ...T> using persistent_set = std::set<T...>;

template<typename ValueType>
class ordered_stuff {
private:
  typedef uint64_t idx_type;
  typedef uint32_t num_bits_type;
  struct entry {
    entry(idx_type idx, ValueType const& contents):idx(idx),contents(contents){}
    idx_type idx;
    ValueType contents;
  };
public:
  struct entry_ref {
  public:
    entry_ref():data(nullptr){}
    bool operator<(entry_ref const& o)const { return data->idx < o.data->idx; }
    ValueType& operator*()const { return data->contents; }
    ValueType* operator->()const { return &data->contents; }
  private:
    entry_ref(entry* data):data(data){}
    entry* data;
    friend class ordered_stuff;
  };
  // TODO: emplace?
  entry_ref insert_before(entry_ref ref, ValueType const& new_contents) {
    std::pair<idx_type,idx_type> p = make_room_for_split(ref.data->idx & ~3, 0, ref.data->idx & 3);
    move_entry(idx, p.second);
    return place_at(p.first, new_contents);
  }
  entry_ref insert_after(entry_ref ref, ValueType const& new_contents) {
    std::pair<idx_type,idx_type> p = make_room_for_split(ref.data->idx & ~3, 0, ref.data->idx & 3);
    move_entry(idx, p.first);
    return place_at(p.second, new_contents);
  }
private:
  // TODO: a custom hashtable for this so we don't need the extra layer of pointers
  std::unordered_map<idx_type, std::unique_ptr<entry>> data;
  bool idx_exists(idx_type idx)const { return data.find(idx) != data.end(); }
  
  // The data forms - mathematically, but not in memory - a lenient B-tree with 2,3,or 4 children for each node.
  // When it overflows to 5, it splits into a 3-node and a 2-node.
  // An index's Nth bit-pair indicates which child it is at the Nth level of the tree (from the leaves).
  // Since a node's first child always exists, parent nodes (which have no data)
  // are represented by the ID of the first descendant leaf, plus their size/level.
  std::pair<idx_type,idx_type> make_room_for_split(idx_type prefix, num_bits_type child_shift, idx_type which_child) {
    idx_type offset = 0;
    const idx_type child_size = 1 << child_shift;
    const idx_type which_child_mask = child_size * 0x3;
    if (idx_exists(prefix+3*child_size)) {
      // split this node to make room for splitting children
      std::pair<idx_type,idx_type> p = make_room_for_split(prefix & ~which_child_mask, child_shift + 2, (prefix & which_child_mask) >> child_shift);
      
      if (which_child <= 2) { move_subtree(prefix + 3*child_size, child_shift-2, p.second + 1*child_size); }
      if (which_child <= 1) { move_subtree(prefix + 2*child_size, child_shift-2, p.second               ); }
      if (which_child <= 0) { move_subtree(prefix + 1*child_size, child_shift-2, p.first  + 2*child_size); }
      
      if (which_child == 0) {
        return std::pair<idx_type,idx_type>(p.first, p.first+child_size);
      }
      else if (which_child == 1) {
        return std::pair<idx_type,idx_type>(p.first+1*child_size, p.first+2*child_size);
      }
      else if (which_child == 2) {
        return std::pair<idx_type,idx_type>(p.first+2*child_size, p.second);
      }
      else if (which_child == 3) {
        return std::pair<idx_type,idx_type>(p.second, p.second+1*child_size);
      }
      else {
        assert(false);
      }
    }

    assert (which_child <= 2);
    return std::pair<idx_type,idx_type>(prefix+which_child*child_size, prefix+(which_child+1)*child_size);
  }
  void move_subtree(idx_type prefix, num_bits_type child_shift, idx_type new_prefix) {
    const idx_type child_size = 1 << child_shift;
    for (int i = 3; i >= 0; --i) {
      const idx_type child_prefix = prefix + child_size*i;
      if (child_shift == 0) {
        move_entry(child_prefix, new_prefix+i);
      }
      else {
        if (idx_exists(child_prefix)) {
          move_subtree(child_prefix, child_shift - 2, new_prefix + child_size*i);
        }
      }
    }
  }
  void move_entry(idx_type idx, idx_type new_idx) {
    if (new_idx != idx) {
      auto i = data.find(idx);
      if (i != data.end()) {
        auto p = data.insert(std::pair<idx_type, std::unique_ptr<entry>>(idx, nullptr));
        assert(p.second);
        p.first->second.swap(i->second);
        p.first->second->idx = idx;
        data.erase(i);
      }
    }
  }
  entry_ref place_at(idx_type idx, ValueType const& new_contents) {
    auto p = data.insert(std::pair<idx_type, std::unique_ptr<entry>>(idx, new entry(idx, new_contents)));
    assert(p.second);
    return entry_ref(p.first->second.get());
  }
};

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
  operator bool()const {
    return (*this) != null();
  }
  
  friend inline std::ostream& operator<<(std::ostream& os, siphash_id const& s) {
    os << s.data_[0] << "," << s.data_[1];
    return os;
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

template<typename TimeType = int64_t, TimeType Never = std::numeric_limits<TimeType>::min(), TimeType MinTime = Never+1, class CombineHashWithTimeTypeFuncType = combine_hash_with_integer<TimeType>>
struct time_type_info {
  static_assert(Never < MinTime, "Never must be less than MinTime");
  typedef TimeType time_type;
  static const TimeType never = Never;
  static const TimeType min_time = MinTime;
  typedef CombineHashWithTimeTypeFuncType combine_hash_with_time_type_func_type;
};

typedef siphash_id entity_id;
typedef siphash_id trigger_id;
const entity_id global_object_id = siphash_id::least();

typedef size_t field_id;
template<typename T>
using optional = boost::optional<T>;
using boost::none;

namespace fields_list_impl {
  class fields_list_nature_base {};
  class empty_fields_list_nature : public fields_list_nature_base {};
  class nonempty_fields_list_nature : public fields_list_nature_base {};
  class generic_input_nature : public fields_list_nature_base {};
  
  struct two { char c[2]; };
  template<class T> std::enable_if_t<std::is_base_of<fields_list_nature_base, typename T::fields_list_nature>::value, char> test(int);
  template<class T> two test(...);
  template<class T, bool is_specified> struct get_fields_list_nature_impl { typedef typename T::fields_list_nature type; };
  template<class T> struct get_fields_list_nature_impl<T, false> { typedef generic_input_nature type; };
  template<class T> struct get_fields_list_nature { typedef typename get_fields_list_nature_impl<T, sizeof(test<T>(0))==1>::type type; };
  
  
  template<typename ...Input> class make_fields_list;
  template<typename HeadNature, typename ...Input> class make_fields_list_contents;
  
  class empty_fields_list {
  public:
    typedef empty_fields_list_nature fields_list_nature;
    static const size_t size = 0;
    //template<typename T> static constexpr field_id idx_of() { static_assert(false, "No field of that type exists"); }
    // TODO find a way to make invalid-field errors more readable
  };
  template<typename Head, typename ...Tail>
  class nonempty_fields_list {
  public:
    typedef nonempty_fields_list_nature fields_list_nature;
    typedef Head head;
    typedef typename make_fields_list<Tail...>::type tail;
    static const field_id idx = tail::size;
    static const size_t size = tail::size + 1;
    template<typename T> static constexpr std::enable_if_t< std::is_same<T, head>::value, field_id> idx_of() { return idx; }
    template<typename T> static constexpr std::enable_if_t<!std::is_same<T, head>::value, field_id> idx_of() { return tail::template idx_of<T>(); }
  };
  
  template<typename ...Input>
  struct make_fields_list_contents<   empty_fields_list_nature,      Input...> { typedef    empty_fields_list                type; };
  template<typename Head, typename ...Tail>
  struct make_fields_list_contents<       generic_input_nature, Head, Tail...> { typedef nonempty_fields_list<Head, Tail...> type; };
  template<typename Head, typename ...Tail>
  struct make_fields_list_contents<nonempty_fields_list_nature, Head, Tail...> {
    typedef typename make_fields_list<typename Head::head, typename Head::tail, Tail...>::type type;
  };
  
  template<>
  struct make_fields_list<> { typedef empty_fields_list type; };
  template<typename Head, typename ...Tail>
  struct make_fields_list<Head, Tail...> { typedef typename make_fields_list_contents<typename get_fields_list_nature<Head>::type, Head, Tail...>::type type; };
  

  template<class FieldsList, template<typename> class repeated>
  class foreach_field {
    typedef typename FieldsList::head head;
    repeated<head> head_;
    foreach_field<typename FieldsList::tail, repeated> tail_;
  public:
    template<typename T> inline std::enable_if_t< std::is_same<T, head>::value, repeated<T>&> get() { return head_; }
    template<typename T> inline std::enable_if_t<!std::is_same<T, head>::value, repeated<T>&> get() { return tail_.get<T>(); }
    template<typename T> inline std::enable_if_t< std::is_same<T, head>::value, repeated<T> const&> get()const { return head_; }
    template<typename T> inline std::enable_if_t<!std::is_same<T, head>::value, repeated<T> const&> get()const { return tail_.get<T>(); }
  };
  template<template<typename> class repeated>
  class foreach_field<empty_fields_list, repeated> {
    //template<typename T> inline repeated<T>& get() { static_assert(false, "No field of that type exists"); }
    //template<typename T> inline repeated<T> const& get()const { static_assert(false, "No field of that type exists"); }
  };
  
  template<class FieldsList, typename repeated>
  class foreach_field_array : public std::array<repeated, FieldsList::size> {
  public:
    template<typename T> inline repeated& get() { return (*this)[FieldsList::template idx_of<T>()]; }
    template<typename T> inline repeated const& get()const { return (*this)[FieldsList::template idx_of<T>()]; }
  };
  
  template<class FieldsList, class FuncClass>
  class function_array : public foreach_field_array<FieldsList, decltype(&FuncClass::template func<typename FieldsList::head>)> {
  public:
    function_array() { add_fptr<FieldsList>(); }
  private:
    template<class SubList> inline std::enable_if_t<!std::is_same<SubList, empty_fields_list>::value> add_fptr() {
      foreach_field_array<FieldsList, decltype(&FuncClass::template func<typename FieldsList::head>)>::template get<typename SubList::head>() = &FuncClass::template func<typename SubList::head>;
      add_fptr<typename SubList::tail>();
    }
    template<class SubList> inline std::enable_if_t< std::is_same<SubList, empty_fields_list>::value> add_fptr() {}
  };
}
template<typename ...Input> using fields_list = typename fields_list_impl::make_fields_list<Input...>::type;
struct entity_field_id {
  entity_field_id(entity_id e, field_id f):e(e),f(f){}
  entity_id e;
  field_id f;
  bool operator==(entity_field_id const& o)const { return (e == o.e) && (f == o.f); }
};

} //end namespace time_steward_system
namespace std {
  template<>
  class hash<time_steward_system::entity_field_id> {
    public:
    size_t operator()(time_steward_system::entity_field_id const& i)const {
      return std::hash<time_steward_system::siphash_id>()(i.e) + i.f;
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
    void modified(time_steward_accessor& acc, entity_field_id const& id) {
      if (!ever_modified) {
        ever_modified = true;
        acc.entity_fields_modified.push_back(id);
      }
    }
  };
  
  template<typename Field>
  using field_info = optional<Field>;
  /*struct field_info {
    Field data;
  };*/
    
  struct entity_info {
    entity_id id;
    fields_list_impl::foreach_field<entity_fields, field_info> fields;
    fields_list_impl::foreach_field_array<entity_fields, field_metadata> metadata;
  };
  
public:
  struct entity_ref {
  public:
    entity_ref():data(nullptr){}
    operator bool()const { return bool(data); }
    entity_id id()const { return data->id; }
  private:
    entity_info* data;
    entity_ref(entity_info* data):data(data){}
    friend class time_steward_accessor;
  };
  
private:
  template<typename Field>
  inline optional<Field>& get_impl(entity_ref e)const {
    const field_id idx = entity_fields::template idx_of<Field>();
    optional<Field>& result = e.data->fields.get<Field>();
    const entity_field_id id(e.id(), idx);
    if (e.data->metadata[idx].accessed(*this, id)) {
      result = ts_->template get_provisional_entity_field_before<Field>(e.id(), time_);
    }
    return result;
  }
  
public:
  inline entity_ref get(entity_id id)const {
    entity_info& e = entities[id];
    e.id = id;
    return entity_ref(&e);
  }
  
  template<typename Field> inline optional<Field> const& get    (entity_ref e)const { return get_impl<Field>(e); }
  template<typename Field> inline optional<Field>      & get_mut(entity_ref e) {
    optional<Field>& result = get_impl<Field>(e);
    const field_id idx = entity_fields::template idx_of<Field>();
    e.data->metadata[idx].modified(*this, entity_field_id(e.id(), idx));
    return result;
  }
  template<typename Field>
  inline optional<Field>& set(entity_ref e, optional<Field> new_contents) {
    const field_id idx = entity_fields::template idx_of<Field>();
    e.data->metadata[idx].modified(*this, entity_field_id(e.id(), idx));
    optional<Field>& result = e.data->fields.get<Field>();
    result = new_contents;
    return result;
  }
  
  void anticipate_event(time_type when, std::shared_ptr<const event> e) {
    assert(is_trigger_); // TODO an exception
    assert(when >= time_.base_time); // TODO an exception
    new_upcoming_events.push_back(std::make_pair((when == time_.base_time) ? extended_time(time_, create_id()) : extended_time(when, create_id()), e));
  };
  void set_trigger(trigger_id id, std::shared_ptr<const trigger> t) {
    trigger_changes[id] = t;
  };
  trigger_id set_trigger(std::shared_ptr<const trigger> t) {
    trigger_id id = create_id();
    set_trigger(id, t);
    return id;
  };
  time_type now()const {
    return time_.base_time;
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
    
  time_steward_accessor(time_steward_accessor const&) = delete;
  time_steward_accessor(time_steward_accessor&&) = delete;
  time_steward_accessor& operator=(time_steward_accessor const&) = delete;
  time_steward_accessor& operator=(time_steward_accessor&&) = delete;
private:
  TimeSteward const* ts_;
  extended_time time_;
  bool is_trigger_;
  size_t ids_created_;
  mutable std::unordered_map<entity_id, entity_info> entities;
  
  std::unordered_map<trigger_id, std::shared_ptr<const trigger>> trigger_changes;
  std::vector<std::pair<extended_time, std::shared_ptr<const event>>> new_upcoming_events;
  std::vector<entity_field_id> entity_fields_modified;
  mutable std::vector<entity_field_id> entity_fields_preexisting_state_accessed;

  siphash_id create_id() { return siphash_id(time_.id(), ids_created_++); }
    
  time_steward_accessor(TimeSteward const* ts, extended_time const& time, bool is_trigger):ts_(ts),time_(time),is_trigger_(is_trigger),ids_created_(0){}
  void process_event(event const* e) {
    (*e)(this);
  }
  friend TimeSteward;
};

namespace impl {
template<typename TimeTypeInfo>
struct extended_time {
  typedef typename TimeTypeInfo::time_type time_type;

  time_type base_time;
  // We construct extended_times with new unique ids;
  // some of them extend old extended_times, so tiebreakers.front() is not unique among extended_times.
  // However, tiebreakers.back() is unique among extended_times.
  // TODO: a structure with better copy asymptotics than std::vector and better constant speed for short values.
  std::vector<siphash_id> tiebreakers;
  uint32_t non_trigger_detail_tiebreakers_size;
  siphash_id id()const { return tiebreakers.back(); }
  
  struct first_t{}; struct last_t{}; static const first_t first; static const last_t last;
  extended_time():base_time(TimeTypeInfo::never),non_trigger_detail_tiebreakers_size(1) { tiebreakers.push_back(siphash_id()); }
  extended_time(time_type base_time, first_t):base_time(base_time),non_trigger_detail_tiebreakers_size(1) { tiebreakers.push_back(siphash_id::least()); }
  extended_time(time_type base_time,  last_t):base_time(base_time),non_trigger_detail_tiebreakers_size(1) { tiebreakers.push_back(siphash_id::greatest()); }
  extended_time(time_type base_time, siphash_id tiebreaker):base_time(base_time),non_trigger_detail_tiebreakers_size(1) { tiebreakers.push_back(tiebreaker); }
  extended_time(extended_time base_exttime, siphash_id further_tiebreaker)
      :
      base_time(base_exttime.base_time),
      tiebreakers(base_exttime.tiebreakers),
      non_trigger_detail_tiebreakers_size(base_exttime.non_trigger_detail_tiebreakers_size)
  {
    tiebreakers.resize(non_trigger_detail_tiebreakers_size);
    tiebreakers.push_back(further_tiebreaker);
  }
  
  bool operator==(extended_time const& o)const { return id() == o.id(); }
  bool operator!=(extended_time const& o)const { return id() != o.id(); }
  bool operator<(extended_time const& o)const {
    if (base_time != o.base_time) { return base_time < o.base_time; }
    const bool tdts_unequal = non_trigger_detail_tiebreakers_size != o.non_trigger_detail_tiebreakers_size;
    for (size_t i = 0; ; ++i) {
      if (tdts_unequal && i == o.non_trigger_detail_tiebreakers_size) { return false; }
      if (tdts_unequal && i == non_trigger_detail_tiebreakers_size) { return true; }
      if (i == o.tiebreakers.size()) { return false; }
      if (i == tiebreakers.size()) { return true; }
      if (tiebreakers[i] != o.tiebreakers[i]) { return tiebreakers[i] < o.tiebreakers[i]; }
    }
  }
  bool operator>(extended_time const& o)const { return o < *this; }
  bool operator>=(extended_time const& o)const { return !(*this < o); }
  bool operator<=(extended_time const& o)const { return !(o < *this); }
};
}

} //end namespace time_steward_system
namespace std {
  template<typename TimeTypeInfo>
  class hash<time_steward_system::impl::extended_time<TimeTypeInfo>> {
    public:
    size_t operator()(time_steward_system::impl::extended_time<TimeTypeInfo> const& t)const {
      return std::hash<time_steward_system::siphash_id>()(t.id());
    }
  };
}
namespace time_steward_system {

template<class FieldsList/*, class Event = */, class TimeTypeInfo = time_type_info<>>
class time_steward {
public:
  typedef FieldsList entity_fields;
  typedef time_steward_accessor<time_steward> accessor;
  friend class time_steward_accessor<time_steward>;
  
  typedef typename TimeTypeInfo::time_type time_type;
  static const time_type min_time = TimeTypeInfo::min_time;
  static const time_type never = TimeTypeInfo::never;
  
  // TODO can the action/event/trigger stuff be template parameters rather than always being virtual classes to be subclassed?
  class event {
  public:
    virtual void operator()(accessor* accessor)const = 0;
    virtual ~event(){}
  };
  typedef event trigger;
private:
  typedef typename TimeTypeInfo::combine_hash_with_time_type_func_type combine_hash_with_time_type_func_type;
  
  static const uint32_t not_a_trigger = std::numeric_limits<uint32_t>::max();
  struct extended_time_metadata {
    extended_time_metadata(){}
    extended_time_metadata(siphash_id id):id(id),trigger_iteration(not_a_trigger){}
    extended_time_metadata(siphash_id id, uint32_t trigger_iteration, extended_time closest_non_trigger_ancestor):
      id(id),trigger_iteration(trigger_iteration),closest_non_trigger_ancestor(closest_non_trigger_ancestor){}
    
    // TODO consolidate
    std::set<extended_time_metadata, extended_time> children; // usually empty; optimize for size when empty
    
    siphash_id id;
    uint32_t trigger_iteration;
    extended_time closest_non_trigger_ancestor;
    
    // note: comparison only valid between siblings
    bool operator==(extended_time_metadata const& o)const {
      return trigger_iteration == o.trigger_iteration && id == o.id;
    }
    bool operator<(extended_time_metadata const& o)const {
      if (trigger_iteration != o.trigger_iteration) return trigger_iteration < o.trigger_iteration;
      return id < o.id;
    }
  };
  
  typedef ordered_stuff<extended_time_metadata>::entry_ref extended_time;
  static ordered_stuff<extended_time_metadata> extended_times;
  static std::map<time_type, extended_time> base_time_roots;
  
  static extended_time get_base_time_root(time_type t) {
    if (base_time_roots.empty()) {
      // create a sentinel with no children at the end
      base_time_roots.emplace(max_time, extended_times.insert_only(extended_time_metadata()));
    }
    const auto i = base_time_roots.lower_bound(t);
    if (i->first == t) { return i->second; }
    assert(i != base_time_roots.end());
    const auto j = base_times.emplace_hint(i, t, extended_times.insert_before(i->second, extended_time_metadata()));
    return j->second;
  }
  static extended_time make_extended_time_impl(extended_time t, extended_time_metadata meta) {
    if (t.children.empty()) {
      // create a sentinel with no children at the end
      t.children.emplace(siphash_id::last(), extended_times.insert_after(t, extended_time_metadata(siphash_id::last())));
    }
    const auto i = t.children.lower_bound(meta);
    assert(i != t.children.end());
    if (i->first == meta) { return i->second; }
    extended_time result = extended_times.insert_before(i->second, meta);
    const auto j = t.children.emplace_hint(i, meta, result);
    j->second = result;
    return result;
  }
  static extended_time make_extended_time(time_type t, siphash_id id) {
    return make_extended_time_impl(get_base_time(t), id);
  }
  static extended_time make_extended_time(extended_time t, siphash_id id) {
    return make_extended_time_impl(t->closest_non_trigger_ancestor ? t->closest_non_trigger_ancestor : t, id);
  }
  static extended_time trigger_call_time(trigger_id tid, extended_time when_triggered) {
    const extended_time t = when_triggered->closest_non_trigger_ancestor ? when_triggered->closest_non_trigger_ancestor : when_triggered;
    const siphash_id id(t->id, tid, when_triggered->trigger_iteration);
    const uint32_t trigger_iteration = (when_triggered->trigger_iteration == not_a_trigger) ? 0 :
      (when_triggered->trigger_iteration + (when_triggered->id >= id) ? 1 : 0);
    return make_extended_time_impl(t, extended_time_metadata(id, trigger_iteration, t)));
  }
  
  struct trigger_call_info {
    trigger_call_info():field_changes_andor_creations_triggering_this(0){}
    uint32_t field_changes_andor_creations_triggering_this;
    std::set<extended_time> anticipated_events;
  };
  struct event_pile_info {
    event_pile_info(std::shared_ptr<const event> instigating_event, trigger_id tid = trigger_id()) :
      instigating_event (instigating_event),
      tid(tid),
      creation_cut_off (0),
      has_been_executed (false)
      {}
    std::shared_ptr<const event> instigating_event;
    trigger_id tid;
    std::unordered_set<entity_field_id> entity_fields_pile_accessed;
    std::unordered_set<entity_field_id> entity_fields_pile_modified;
    std::unordered_set<trigger_id> triggers_changed;
    bool creation_cut_off;
    bool has_been_executed;
    bool should_be_executed()const { return instigating_event && !creation_cut_off; }
  };
  
  // Hack, TODO fix: Non-static so that C++ doesn't make it a compile-time constant.
  // (We need to return references into it.)
  /*static*/ const fields_list_impl::foreach_field<entity_fields, optional> field_absent;
  
  template<typename Field>
  using field_throughout_time = std::map<extended_time, optional<Field>>;
  struct field_metadata_throughout_time {
    std::map<extended_time, persistent_set<trigger_id>> triggers_pointing_at_this_changes;
    std::set<extended_time> event_piles_which_accessed_this;
  };
  struct entity_throughout_time_info {
    fields_list_impl::foreach_field<entity_fields, field_throughout_time> fields;
    fields_list_impl::foreach_field_array<entity_fields, field_metadata_throughout_time> metadata;
  };
  typedef std::map<extended_time, trigger_call_info> trigger_throughout_time_info;
  
  extended_time trigger_call_time(trigger_id id, extended_time const& when_triggered) {
    extended_time result = when_triggered;
    for (size_t i = result.non_trigger_detail_tiebreakers_size-1; ; ++i) {
      trigger_id next_tiebreaker(id, result.tiebreakers[i]);
      if (i+1 == result.tiebreakers.size() || next_tiebreaker > result.tiebreakers[i+1]) {
        result.tiebreakers.resize(i+2);
        result.tiebreakers[i+1] = next_tiebreaker;
        return result;
      }
    }
  }
  void update_trigger(bool undoing, trigger_id tid, extended_time const& field_change_andor_creation_time, bool force_trigger_change = false, std::shared_ptr<const trigger> new_trigger = nullptr) {
    trigger_throughout_time_info& trigger_info = triggers[tid];
    const auto next_call_iter = trigger_info.upper_bound(field_change_andor_creation_time); // upper_bound: if we're IN a trigger call, we still want to trigger it again
    if (next_call_iter != trigger_info.begin()) {
      const auto cut_off_call_iter = boost::prior(next_call_iter);
      const auto end_iter2 = (next_call_iter == trigger_info.end()) ?
        cut_off_call_iter->second.anticipated_events.end() : 
        cut_off_call_iter->second.anticipated_events.lower_bound(next_call_iter->first); // upper/lower shouldn't matter: anticipated events are never triggers
      for (auto i = cut_off_call_iter->second.anticipated_events.upper_bound(field_change_andor_creation_time); i != end_iter2; ++i) {
        const auto pile_iter = event_piles.find(*i);
        if (undoing) {
          pile_iter->second.creation_cut_off = false;
          if (pile_iter->second.should_be_executed()) { event_piles_not_correctly_executed.insert(pile_iter->first); }
        }
        else {
          pile_iter->second.creation_cut_off = true;
          if (pile_iter->second.has_been_executed) { event_piles_not_correctly_executed.insert(pile_iter->first); }
          else                                     { event_piles_not_correctly_executed.erase (pile_iter->first); }
        }
      }
      if (!undoing && !force_trigger_change) { new_trigger = event_piles.find(cut_off_call_iter->first)->second.instigating_event; }
    }
    
    const extended_time new_trigger_call_time = trigger_call_time(tid, field_change_andor_creation_time);
    if (undoing) {
      const auto i = trigger_info.find(new_trigger_call_time); assert(i != trigger_info.end());
      --i->second.field_changes_andor_creations_triggering_this;
      if (i->second.field_changes_andor_creations_triggering_this == 0) {
        trigger_info.erase(i);
        erase_instigating_event(new_trigger_call_time);
      }
    }
    else {
      // TODO minor: fix duplicate code (id TG0+Xm06xcLdjQ)
      const auto p = trigger_info.insert(std::make_pair(new_trigger_call_time, trigger_call_info()));
      if (p.second && new_trigger) {
        insert_instigating_event(new_trigger_call_time, event_pile_info(new_trigger, tid));
      }
      ++p.first->second.field_changes_andor_creations_triggering_this;
    }
  }
  void update_triggers(bool undoing, entity_throughout_time_info& dst, field_id id, extended_time const& field_change_time) {
    const auto next_triggers_iter = dst.metadata[id].triggers_pointing_at_this_changes.lower_bound(field_change_time);
    if (next_triggers_iter == dst.metadata[id].triggers_pointing_at_this_changes.begin()) return;
    const auto triggers_iter = boost::prior(next_triggers_iter);
    for (auto tid : triggers_iter->second) {
      update_trigger(undoing, tid, field_change_time);
    }
  }
  
  struct copy_field_change_from_accessor {
    template<typename Field>
    static void func(time_steward* ts, extended_time const& time, decltype(accessor::entity_info::fields)& src, entity_throughout_time_info& dst) {
      const auto p = dst.fields.template get<Field>().insert(std::make_pair(time, src.template get<Field>()));
      assert(p.second);
      ts->update_triggers(false, dst, entity_fields::template idx_of<Field>(), time);
    }
  };
  struct undo_field_change {
    template<typename Field>
    static void func(time_steward* ts, extended_time const& time, entity_throughout_time_info& dst) {
      const auto change_iter = dst.fields.template get<Field>().find(time);
      assert (change_iter != dst.fields.template get<Field>().end());
      ts->update_triggers(true, dst, entity_fields::template idx_of<Field>(), time);
      dst.fields.template get<Field>().erase(change_iter);
    }
  };
  
  // Hack, TODO fix: Non-static so that C++ doesn't make it a compile-time constant.
  /*static*/ const fields_list_impl::function_array<entity_fields, copy_field_change_from_accessor> copy_field_change_from_accessor_funcs;
  /*static*/ const fields_list_impl::function_array<entity_fields, undo_field_change> undo_field_change_funcs;
  
  // The events map doesn't need to be ordered (even though it has a meaningful order)
  // because it's just for looking up events whose times we know.
  typedef std::unordered_map<extended_time, event_pile_info> event_piles_map;
  typedef std::unordered_map<entity_id, entity_throughout_time_info> entities_map;
public:
  time_steward() {}
  
  void insert_fiat_event(time_type time, uint64_t distinguisher, std::shared_ptr<const event> e) {
    // This function is one of exactly two places where
    // (persistent) extended_times are created.
    // Uniqueness justification:
    // TODO
    const extended_time t(time, combine_hash_with_time_type_func_type()(siphash_id(distinguisher), time));
    // TODO throw an exception if the user inserts two events at the same time with the same distinguisher
    insert_instigating_event(t, event_pile_info(e));
  }
  void erase_fiat_event(time_type time, uint64_t distinguisher) {
    const extended_time t(time, siphash_id(distinguisher));
    erase_instigating_event(t);
  }
  
  // For external client code to examine the state - 
  //   e.g., for display.
  // Output of these should never find their way into internal client code,
  //   and it's a little strange (but ok) to have them affect FiatEvents.
  /*template<typename T>
  entity_ref<T const> get_entity_data_after(entity_id<T> const& id, time_type const& time) {
    // Note: For collision safety, these extended_times must not persist.
    return dynamic_pointer_cast<T const>(entity_ref<entity const>(
      get_actual_entity_data_before(id, extended_time(time, extended_time::last))));
  }*/
  std::unique_ptr<accessor> accessor_after(time_type const& time) {
    extended_time et(time, extended_time::last);
    update_through_time(et);
    return std::unique_ptr<accessor>(new accessor(this, et, false));
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
  std::set<extended_time> event_piles_not_correctly_executed;
  std::unordered_map<trigger_id, trigger_throughout_time_info> triggers;
  
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
  /*entity const* get_actual_entity_data_before(entity_id<entity> const& id, extended_time const& time) {
    update_until_time(time);
    return get_provisional_entity_data_before(id, time);
  }*/
  template<typename Field>
  optional<Field> const& get_provisional_entity_field_before(entity_id id, extended_time const& time)const {
    const auto entity_stream_iter = entities.find(id);
    if (entity_stream_iter == entities.end()) { return field_absent.template get<Field>(); }
    
    entity_throughout_time_info const& e = entity_stream_iter->second;
    const auto next_change_iter = e.fields.template get<Field>().upper_bound(time);
    if (next_change_iter == e.fields.template get<Field>().begin()) { return field_absent.template get<Field>(); }
    return boost::prior(next_change_iter)->second;
  }
  
  void insert_instigating_event(extended_time const& time, event_pile_info const& e) {
    const auto p1 = event_piles.insert(std::make_pair(time, e));
    assert(p1.second);
    const auto p2 = event_piles_not_correctly_executed.insert(time);
    assert(p2.second);
  }
  void erase_instigating_event(extended_time const& time) {
    const auto event_pile_iter = event_piles.find(time);
    if (event_pile_iter != event_piles.end()) {
      event_pile_info& pile_info = event_pile_iter->second;
      
      pile_info.instigating_event = nullptr;
      if (pile_info.has_been_executed) {
        event_piles_not_correctly_executed.insert(time);
      }
      else {
        event_piles_not_correctly_executed.erase(time);
        event_piles.erase(event_pile_iter);
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
      accessor a(this, time, bool(pile_info.tid));
      a.process_event(pile_info.instigating_event.get());
      
      for (entity_field_id const& id : a.entity_fields_preexisting_state_accessed) {
        pile_info.entity_fields_pile_accessed.insert(id);
        auto& metadata = entities[id.e].metadata[id.f];
        const auto p = metadata.event_piles_which_accessed_this.insert(time);
        assert(p.second);
        if (pile_info.tid) {
          persistent_set<trigger_id> new_triggers;
          const auto next_triggers_iter = metadata.triggers_pointing_at_this_changes.lower_bound(time); // lower/upper bound shouldn't matter because there shouldn't be an entry now.
          if (next_triggers_iter != metadata.triggers_pointing_at_this_changes.begin()) {
            new_triggers = boost::prior(next_triggers_iter)->second;
          }
          new_triggers.insert(pile_info.tid);
          // Hack sZInnn3FkZy0yg: insert even if there's no change, so that the below code can catch it.
          metadata.triggers_pointing_at_this_changes.insert(std::make_pair(time, new_triggers));
        }
      }
      for (entity_field_id const& id : a.entity_fields_modified) {
        pile_info.entity_fields_pile_modified.insert(id);
        (*copy_field_change_from_accessor_funcs[id.f])(this, time, a.entities.find(id.e)->second.fields, entities[id.e]);
      }
      for (std::pair<trigger_id, std::shared_ptr<const trigger>> const& t : a.trigger_changes) {
        pile_info.triggers_changed.insert(t.first);
        update_trigger(false, t.first, time, true, t.second);
      }
      if (pile_info.tid) {
        auto& trigger_info = triggers.find(pile_info.tid)->second;
        auto call_iter = trigger_info.find(time);
        for (std::pair<extended_time, std::shared_ptr<const event>> const& ev : a.new_upcoming_events) {
          insert_instigating_event(ev.first, event_pile_info(ev.second));
          call_iter->second.anticipated_events.insert(ev.first);
        }
        if (call_iter != trigger_info.begin()) {
          for (entity_field_id former_trigger_access : event_piles.find(boost::prior(call_iter)->first)->second.entity_fields_pile_accessed) {
            auto& metadata = entities[former_trigger_access.e].metadata[former_trigger_access.f];
            const auto next_triggers_iter = metadata.triggers_pointing_at_this_changes.lower_bound(time); // lower_bound: we want to find the entry at this time
            if ((next_triggers_iter != metadata.triggers_pointing_at_this_changes.end()) && (next_triggers_iter->first == time)) {
              // Hack sZInnn3FkZy0yg: we inserted above, so don't remove now.
            }
            else {
              assert (next_triggers_iter != metadata.triggers_pointing_at_this_changes.begin());
              persistent_set<trigger_id> new_triggers = boost::prior(next_triggers_iter)->second;
              const auto p2 = new_triggers.erase(pile_info.tid);
              assert (p2);
              metadata.triggers_pointing_at_this_changes.insert(std::make_pair(time, new_triggers));
            }
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
    if (!pile_info.has_been_executed) return false;
    
    for (entity_field_id const& id : pile_info.entity_fields_pile_accessed) {
      auto& metadata = entities.find(id.e)->second.metadata[id.f];
      const auto j = metadata.event_piles_which_accessed_this.erase(time);
      assert(j);
      if (pile_info.tid) {
        metadata.triggers_pointing_at_this_changes.erase(time);
      }
    }
    for (entity_field_id const& id : pile_info.entity_fields_pile_modified) {
      (*undo_field_change_funcs[id.f])(this, time, entities.find(id.e)->second);
    }
    for (trigger_id const& tid : pile_info.triggers_changed) {
      update_trigger(true, tid, time);
    }
    if (pile_info.tid) {
      auto& trigger_info = triggers.find(pile_info.tid)->second;
      auto call_iter = trigger_info.find(time);
      for (extended_time const& t : call_iter->second.anticipated_events) {
        erase_instigating_event(t);
      }
      call_iter->second.anticipated_events.clear();
      if (call_iter != trigger_info.begin()) {
        for (entity_field_id former_trigger_access : event_piles.find(boost::prior(call_iter)->first)->second.entity_fields_pile_accessed) {
          auto& metadata = entities.find(former_trigger_access.e)->second.metadata[former_trigger_access.f];
          metadata.triggers_pointing_at_this_changes.erase(time);
        }
      }
    }
    pile_info.entity_fields_pile_accessed.clear();
    pile_info.entity_fields_pile_modified.clear();
    pile_info.triggers_changed.clear();
    
    pile_info.has_been_executed = false;
    if (pile_info.should_be_executed()) {
      event_piles_not_correctly_executed.insert(time);
    }
    if (!pile_info.instigating_event) {
      event_piles.erase(event_pile_iter);
      return true;
    }
    return false;
  }
};

} //end namespace time_steward_system


