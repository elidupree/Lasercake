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

#ifndef LASERCAKE_UTILS_HPP__
#define LASERCAKE_UTILS_HPP__

#include <array>

template<typename Map>
typename Map::mapped_type* find_as_pointer(Map& m, typename Map::key_type const& k) {
  auto i = m.find(k);
  if(i == m.end()) return nullptr;
  else return &(i->second);
}

template<typename scalar_type> scalar_type divide_rounding_towards_zero(scalar_type dividend, scalar_type divisor)
{
	assert(divisor != 0);
	int abs_result = std::abs(dividend) / std::abs(divisor);
	if ((dividend > 0 && divisor > 0) || (dividend < 0 && divisor < 0)) return abs_result;
	else return -abs_result;
}

template<typename scalar_type> class vector3 {
public:
	scalar_type x, y, z;
	vector3():x(0),y(0),z(0){}
	vector3(scalar_type x, scalar_type y, scalar_type z):x(x),y(y),z(z){}
	template<typename OtherType> explicit vector3(vector3<OtherType> const& other):
	  x(other.x),y(other.y),z(other.z){}
	
	// Note: The operators are biased towards the type of the left operand (e.g. vector3<int> + vector3<int64_t> = vector3<int>)
	template<typename OtherType> vector3 operator+(vector3<OtherType> const& other)const {
		return vector3(x + other.x, y + other.y, z + other.z);
	}
	template<typename OtherType> void operator+=(vector3<OtherType> const& other) {
		x += other.x; y += other.y; z += other.z;
	}
	template<typename OtherType> vector3 operator-(vector3<OtherType> const& other)const {
		return vector3(x - other.x, y - other.y, z - other.z);
	}
	template<typename OtherType> void operator-=(vector3<OtherType> const& other) {
		x -= other.x; y -= other.y; z -= other.z;
	}
	vector3 operator*(scalar_type other)const {
		return vector3(x * other, y * other, z * other);
	}
	void operator*=(scalar_type other) {
		x *= other; y *= other; z *= other;
	}
	vector3 operator/(scalar_type other)const {
		return vector3(divide_rounding_towards_zero(x, other), divide_rounding_towards_zero(y, other), divide_rounding_towards_zero(z, other));
	}
	void operator/=(scalar_type other) {
		x = divide_rounding_towards_zero(x, other); y = divide_rounding_towards_zero(y, other); z = divide_rounding_towards_zero(z, other);
	}
	vector3 operator-()const { // unary minus
		return vector3(-x, -y, -z);
	}
	bool operator==(vector3 const& other)const {return x == other.x && y == other.y && z == other.z; }
	bool operator!=(vector3 const& other)const {return x != other.x || y != other.y || z != other.z; }
	
	// Do not try to use this if either vector has an unsigned scalar_type. It might work in some situations, but why would you ever do that anyway?
	// You are required to specify an output type, because of the risk of overflow. Make sure to choose one that can fit the squares of the numbers you're dealing with.
	template<typename OutputType, typename OtherType> OutputType dot(vector3<OtherType> const& other)const {
		return (OutputType)x * (OutputType)other.x +
		       (OutputType)y * (OutputType)other.y +
		       (OutputType)z * (OutputType)other.z;
	}
	
	// Warning - might be slightly inaccurate (in addition to the integer rounding error) for very large vectors, due to floating-point inaccuracy.
	scalar_type magnitude()const { return (scalar_type)std::sqrt(dot<double>(*this)); }
	
	// Choose these the way you'd choose dot's output type (see the comment above)
	// we had trouble making these templates, so now they just always use int64_t
	bool magnitude_within_32_bits_is_less_than(scalar_type amount)const {
	  return dot<int64_t>(*this) < (int64_t)amount * (int64_t)amount;
	}
	bool magnitude_within_32_bits_is_greater_than(scalar_type amount)const {
	  return dot<int64_t>(*this) > (int64_t)amount * (int64_t)amount;
	}
	bool operator<(vector3 const& other)const { return (x < other.x) || ((x == other.x) && ((y < other.y) || ((y == other.y) && (z < other.z)))); }
};

namespace std {
  template<typename scalar_type> struct hash<vector3<scalar_type> > {
    inline size_t operator()(vector3<scalar_type> const& v) const {
      size_t seed = 0;
      boost::hash_combine(seed, v.x);
      boost::hash_combine(seed, v.y);
      boost::hash_combine(seed, v.z);
      return seed;
    }
  };
}

typedef int8_t neighboring_tile_differential;
typedef int8_t cardinal_direction_index;
const cardinal_direction_index NUM_CARDINAL_DIRECTIONS = 6;
struct cardinal_direction {
  cardinal_direction(vector3<neighboring_tile_differential> v, cardinal_direction_index i):v(v),cardinal_direction_idx(i){}
  vector3<neighboring_tile_differential> v;
  cardinal_direction_index cardinal_direction_idx;
  cardinal_direction operator-()const;
};

template<typename scalar_type> inline vector3<scalar_type> project_onto_cardinal_direction(vector3<scalar_type> src, cardinal_direction dir) {
  return vector3<scalar_type>(src.x * std::abs((scalar_type)dir.v.x), src.y * std::abs((scalar_type)dir.v.y), src.z * std::abs((scalar_type)dir.v.z));
}

const vector3<neighboring_tile_differential> xunitv(1, 0, 0);
const vector3<neighboring_tile_differential> yunitv(0, 1, 0);
const vector3<neighboring_tile_differential> zunitv(0, 0, 1);
// the order of this must be in sync with the order of hacky_vector_indexing_internals::cardinal_direction_vector_to_index
const cardinal_direction cdir_xminus = cardinal_direction(-xunitv, 0);
const cardinal_direction cdir_yminus = cardinal_direction(-yunitv, 1);
const cardinal_direction cdir_zminus = cardinal_direction(-zunitv, 2);
const cardinal_direction cdir_xplus = cardinal_direction(xunitv, 3);
const cardinal_direction cdir_yplus = cardinal_direction(yunitv, 4);
const cardinal_direction cdir_zplus = cardinal_direction(zunitv, 5);
const cardinal_direction cardinal_directions[NUM_CARDINAL_DIRECTIONS] = { cdir_xminus, cdir_yminus, cdir_zminus, cdir_xplus, cdir_yplus, cdir_zplus };
#define EACH_CARDINAL_DIRECTION(varname) cardinal_direction varname : cardinal_directions
inline cardinal_direction cardinal_direction::operator-()const { return cardinal_directions[(cardinal_direction_idx + 3)%6]; }

template<typename value_type> class value_for_each_cardinal_direction {
public:
  value_for_each_cardinal_direction& operator=(value_for_each_cardinal_direction const& other){
    for(cardinal_direction_index dir_idx=0; dir_idx < NUM_CARDINAL_DIRECTIONS; ++dir_idx) {
      data[dir_idx] = other.data[dir_idx];
    }
    return *this;
  }
  value_for_each_cardinal_direction(value_type initial_value) {
    for(cardinal_direction_index dir_idx=0; dir_idx < NUM_CARDINAL_DIRECTIONS; ++dir_idx) {
      data[dir_idx] = initial_value;
    }
  }
  value_type      & operator[](cardinal_direction const& dir) { return data[dir.cardinal_direction_idx]; }
  value_type const& operator[](cardinal_direction const& dir)const { return data[dir.cardinal_direction_idx]; }
private:
  typedef std::array<value_type, NUM_CARDINAL_DIRECTIONS> internal_array;
public:
  typename internal_array::iterator begin() { return data.begin(); }
  typename internal_array::iterator end() { return data.end(); }
  typename internal_array::const_iterator cbegin()const { return data.cbegin(); }
  typename internal_array::const_iterator cend()const { return data.cend(); }
private:
  internal_array data;
};


class bounds_checked_int {
public:
  bounds_checked_int():value(0){}
  bounds_checked_int(int value):value(value){}
  bounds_checked_int &operator=(int other) { value = other; return *this; }
  bounds_checked_int operator+(int other)const {
    assert((int64_t)value + (int64_t)other < (1LL << 31));
    assert((int64_t)value + (int64_t)other > -(1LL << 31));
    return bounds_checked_int(value + other);
  }
  void operator+=(int other) {
    assert((int64_t)value + (int64_t)other < (1LL << 31));
    assert((int64_t)value + (int64_t)other > -(1LL << 31));
    value += other;
  }
  bounds_checked_int& operator++() {
    *this += 1; return *this;
  }
  bounds_checked_int operator++(int) {
    bounds_checked_int result(value);
    *this += 1;
    return result;
  }
  bounds_checked_int& operator--() {
    *this -= 1; return *this;
  }
  bounds_checked_int operator--(int) {
    bounds_checked_int result(value);
    *this -= 1;
    return result;
  }
  bounds_checked_int operator-(int other)const {
    assert((int64_t)value - (int64_t)other < (1LL << 31));
    assert((int64_t)value - (int64_t)other > -(1LL << 31));
    return bounds_checked_int(value - other);
  }
  void operator-=(int other) {
    assert((int64_t)value - (int64_t)other < (1LL << 31));
    assert((int64_t)value - (int64_t)other > -(1LL << 31));
    value -= other;
  }
  bounds_checked_int operator*(int other)const {
    assert((int64_t)value * (int64_t)other < (1LL << 31));
    assert((int64_t)value * (int64_t)other > -(1LL << 31));
    return bounds_checked_int(value * other);
  }
  void operator*=(int other) {
    assert((int64_t)value * (int64_t)other < (1LL << 31));
    assert((int64_t)value * (int64_t)other > -(1LL << 31));
    value *= other;
  }
  bounds_checked_int operator/(int other)const {
    return bounds_checked_int(value / other);
  }
  void operator/=(int other) {
    value /= other;
  }
  operator int()const{ return value; }
private:
  int value;
};

#endif

