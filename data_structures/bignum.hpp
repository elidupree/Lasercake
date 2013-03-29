/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

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

#ifndef LASERCAKE_BIGNUM_HPP__
#define LASERCAKE_BIGNUM_HPP__

// Arbitrary fixed-size integral types.
// We need bit depths that are too large for mere uint64_t but which
// are small enough that register/stack allocation and inlining are
// more efficient than heap allocation and function calls.
// With -O3, most operations unroll to straight-line code with no
// loops or branches.

#include "../config.hpp"
#include <boost/utility/enable_if.hpp>
#include <ostream>


/*
template<size_t Bits>
struct fixed_size_bignum {
  static_assert(Bits % 64 == 0, "er.");
  uint64_t limbs[Bits / 64];
};*/

// can be 32ey later for small platforms

// 'limb' following GMP lib terminology
typedef uint64_t limb_type;
typedef int64_t signed_limb_type;
static const int limb_bits = 64;
typedef DETECTED_uint128_t twice_limb_type;
template<size_t Limbs>
struct bignum {
  // This array's order is little-endian, regardless of the bit order
  // within each word.
  limb_type limbs[Limbs];
};
template<size_t Limbs>
struct bignum_with_overflow {
  bignum<Limbs> num;
  bool overflow;
};
template<size_t LimbsOut, size_t LimbsA, size_t LimbsB>
inline bignum<LimbsOut> long_multiply_unsigned(bignum<LimbsA> a, bignum<LimbsB> b) {
  bignum<LimbsOut> result = {{}};
  for(size_t ia = 0; ia != LimbsA; ++ia) {
    uint32_t carry = 0;
    for(size_t ib = 0; ib != LimbsB; ++ib) {
      //short circuit if a1[i] or a2[j] (or their product?) is zero?
      const size_t ir = ia+ib;
      if(ir+1 < LimbsOut) {
        //full_t z = width_doubling_multiply(a1[i], a2[j]);
        //z[0] z[1]
        const twice_limb_type subproduct = (twice_limb_type)a.limbs[ia] * b.limbs[ib];
        const limb_type low = (limb_type)(subproduct);
        limb_type high = (limb_type)(subproduct >> limb_bits) + carry;
        const limb_type oldlimb0 = result.limbs[ir];
        const limb_type newlimb0 = oldlimb0 + low;
        // Carry.
        // This can't overflow high because e.g. 9*9 == 81 < 99, 0b11 * 0b11 == 0b1001 < 0b1111,
        // max uint32 * max uint32 < max uint64.
        high += (newlimb0 < oldlimb0);
        result.limbs[ir] = newlimb0;
        const limb_type oldlimb1 = result.limbs[ir+1];
        const limb_type newlimb1 = oldlimb1 + high;
        carry = (newlimb1 < oldlimb1);
        result.limbs[ir+1] = newlimb1;
      }
      else if(ir < LimbsOut) {
        limb_type low = a.limbs[ia] * b.limbs[ib];
        result.limbs[ir] += low;
      }
    }
    const size_t ir = ia+LimbsB+1;
    if(ir < LimbsOut) {
      // hasn't been touched yet
      assert(result.limbs[ir] == 0);
      result.limbs[ir] = carry;
    }
  }
  return result;
}
inline bool is_negative_limb(limb_type a) {
  return a & ((limb_type)(1) << (limb_bits-1));
}
template<size_t Limbs>
inline bool is_negative(bignum<Limbs> a) {
  return a.limbs[Limbs-1] & ((limb_type)(1) << (limb_bits-1));
}
// This *could* allow Limbs+1 for carry overflow...

template<size_t Limbs>
inline bignum_with_overflow<Limbs> negate_overflow(bignum<Limbs> a) {
  bignum_with_overflow<Limbs> result;
  result.overflow = false;
  bool carry = true;
  for(size_t ir = 0; ir != Limbs; ++ir) {
    const limb_type newlimb = ~a.limbs[ir] + (limb_type)(carry);
    //LOG << std::hex << "?<" << a.limbs[ir] << ":->" << newlimb << ">";
    result.num.limbs[ir] = newlimb;
    if(ir == Limbs-1 && carry && newlimb == ((limb_type)(1) << (limb_bits-1))) {
      result.overflow = true;
    }
    carry = (carry && (newlimb == 0));
  }
  //LOG << "\n";
  return result;
}
template<size_t Limbs>
inline bignum<Limbs> negate(bignum<Limbs> a) {
  bignum<Limbs> result = negate_overflow(a).num;
  return result;
}
template<size_t LimbsOut, size_t LimbsA, size_t LimbsB>
inline bignum<LimbsOut> long_multiply_signed(bignum<LimbsA> a, bignum<LimbsB> b) {
  static_assert(LimbsA > 0 && LimbsB > 0, "signed ints need a sign bit");
  // If a is negative
  const bool isnega = is_negative(a);
  const bool isnegb = is_negative(b);
  bignum<LimbsOut> result = {{}};
  const bignum_with_overflow<LimbsA> nega = negate_overflow(a);
  const bignum_with_overflow<LimbsB> negb = negate_overflow(b);
  // max negative two's complement int requires special treatment.
  // unspecified zeroes here are implicit from "= {}" of result above.
  if(isnega && nega.overflow) {
    for(size_t ib = 0; ib+LimbsA < LimbsOut && ib < LimbsB; ++ib) {
      const size_t ir = ib+LimbsA;
      result.limbs[ir] = negb.num.limbs[ib];
    }
  }
  else if(isnegb && negb.overflow) {
    for(size_t ia = 0; ia+LimbsB < LimbsOut && ia < LimbsA; ++ia) {
      const size_t ir = ia+LimbsB;
      result.limbs[ir] = nega.num.limbs[ia];
    }
  }
  else {
    const bignum<LimbsA> absa = (isnega ? nega.num : a);
    const bignum<LimbsB> absb = (isnegb ? negb.num : b);
    result = long_multiply_unsigned<LimbsOut>(absa, absb);
    //LOG << "Well?" << (isnega) << ','<<isnegb<<".\n";
    if(isnega != isnegb) {
      result = negate(result);
    }
  }
  return result;
}

template<size_t Limbs>
inline bignum<Limbs> add(bignum<Limbs> a, bignum<Limbs> b) {
  bignum<Limbs> result;
  bool carry = false;
  for(size_t ir = 0; ir != Limbs; ++ir) {
    result.limbs[ir] = (limb_type)(carry);
    carry = false;
    const limb_type oldlimba = result.limbs[ir];
    const limb_type newlimba = oldlimba + a.limbs[ir];
    carry = (carry || newlimba < oldlimba);
    result.limbs[ir] = newlimba;
    const limb_type oldlimbb = result.limbs[ir];
    const limb_type newlimbb = oldlimbb + b.limbs[ir];
    carry = (carry || newlimbb < oldlimbb);
    result.limbs[ir] = newlimbb;
  }
  return result;
}
template<size_t Limbs>
inline bignum<Limbs> subtract(bignum<Limbs> a, bignum<Limbs> b) {
  bignum<Limbs> result;
  bool carry = true;
  for(size_t ir = 0; ir != Limbs; ++ir) {
    result.limbs[ir] = (limb_type)(carry);
    carry = false;
    const limb_type oldlimba = result.limbs[ir];
    const limb_type newlimba = oldlimba + a.limbs[ir];
    carry = (carry || newlimba < oldlimba);
    result.limbs[ir] = newlimba;
    const limb_type oldlimbb = result.limbs[ir];
    const limb_type newlimbb = oldlimbb + ~b.limbs[ir];
    carry = (carry || newlimbb < oldlimbb);
    result.limbs[ir] = newlimbb;
  }
  return result;
}
template<size_t Limbs>
inline bignum<Limbs> bitwise_and(bignum<Limbs> a, bignum<Limbs> b) {
  bignum<Limbs> result;
  for(size_t ir = 0; ir != Limbs; ++ir) {
    result.limbs[ir] = a.limbs[ir] & b.limbs[ir];
  }
  return result;
}
template<size_t Limbs>
inline bignum<Limbs> bitwise_or(bignum<Limbs> a, bignum<Limbs> b) {
  bignum<Limbs> result;
  for(size_t ir = 0; ir != Limbs; ++ir) {
    result.limbs[ir] = a.limbs[ir] | b.limbs[ir];
  }
  return result;
}
template<size_t Limbs>
inline bignum<Limbs> bitwise_xor(bignum<Limbs> a, bignum<Limbs> b) {
  bignum<Limbs> result;
  for(size_t ir = 0; ir != Limbs; ++ir) {
    result.limbs[ir] = a.limbs[ir] ^ b.limbs[ir];
  }
  return result;
}
template<size_t Limbs>
inline bignum<Limbs> bitwise_complement(bignum<Limbs> a) {
  bignum<Limbs> result;
  for(size_t ir = 0; ir != Limbs; ++ir) {
    result.limbs[ir] = ~a.limbs[ir];
  }
  return result;
}
template<size_t Limbs>
inline bool nonzero(bignum<Limbs> a) {
  for(size_t ir = 0; ir != Limbs; ++ir) {
    if(a.limbs[ir]) {
      return true;
    }
  }
  return false;
}
template<size_t Limbs>
inline bool equal(bignum<Limbs> a, bignum<Limbs> b) {
  for(size_t ir = 0; ir != Limbs; ++ir) {
    if(a.limbs[ir] != b.limbs[ir]) {
      return false;
    }
  }
  return true;
}
template<size_t Limbs>
inline bool less_than_unsigned(bignum<Limbs> a, bignum<Limbs> b) {
  for(size_t ir = 0; ir != Limbs; ++ir) {
    const limb_type av = a.limbs[Limbs-1-ir];
    const limb_type bv = b.limbs[Limbs-1-ir];
    if(av < bv) {
      return true;
    }
    if(av > bv) {
      return false;
    }
  }
  return false;
}
template<size_t Limbs>
inline bool less_than_signed(bignum<Limbs> a, bignum<Limbs> b) {
  for(size_t ir = 0; ir != Limbs; ++ir) {
    limb_type av = a.limbs[Limbs-1-ir];
    limb_type bv = b.limbs[Limbs-1-ir];
    if(ir == 0) {
      const limb_type adj = (limb_type)(1) << (limb_bits-1); 
      av ^= adj;
      bv ^= adj;
    }
    if(av < bv) {
      return true;
    }
    if(av > bv) {
      return false;
    }
  }
  return false;
}
#if 0
// TODO is that a good shift argument type?
template<size_t LimbsOut, size_t Limbs>
inline bignum<LimbsOut> shift_right_zero_extend(bignum<Limbs> a, uint32_t shift) {
  bignum<LimbsOut> result;
  size_t limb_offset = shift / limb_bits;
  size_t sub_limb_offset_0 = shift % limb_bits;
  size_t sub_limb_offset_1 = limb_bits - sub_limb_offset_0;
  for(size_t ir = 0; ir != LimbsOut; ++ir) {
    result.limbs[ir] = 0;
    if(ir+limb_offset < Limbs) {
      result.limbs[ir] |= (a.limbs[ir+limb_offset+1] >> sub_limb_offset_0);
      if(ir+limb_offset+1 < Limbs && sub_limb_offset_0 != 0) {
        result.limbs[ir] |= (a.limbs[ir+limb_offset+1] << sub_limb_offset_1);
      }
    }
  }
  return result;
}
#endif
template<bool SignExtend, size_t LimbsOut, size_t Limbs>
inline bignum<LimbsOut> shift_impl(bignum<Limbs> a, ptrdiff_t limb_offset, size_t sub_limb_offset_0) {
  bignum<LimbsOut> result;
  const size_t sub_limb_offset_1 = limb_bits - sub_limb_offset_0;
  const bool neg = is_negative(a);
  const limb_type sign_ext = ((SignExtend && neg) ? (limb_type)(-1) : (limb_type)(0));
  if(sub_limb_offset_0 == 0) {
    for(ptrdiff_t ir = 0; ir != LimbsOut; ++ir) {
      const size_t ia = (size_t)(ir-limb_offset);
      // size_t is modulo, so this also checks for < 0.
      if(ia < Limbs) {
        result.limbs[ir] = a.limbs[ia];
      }
      else if(ir < limb_offset) {
        result.limbs[ir] = 0;
      }
      else {
        result.limbs[ir] = sign_ext;
      }
    }
  }
  else {
    for(ptrdiff_t ir = 0; ir != LimbsOut; ++ir) {
      const size_t ia0 = ir-limb_offset;
      const size_t ia1 = ir-limb_offset+1;
      if(ia0 < Limbs && ia1 < Limbs) {
        result.limbs[ir] =
          (a.limbs[ia0] >> sub_limb_offset_0) |
          (a.limbs[ia1] << sub_limb_offset_1);
      }
      else if(ia0 < Limbs) {
        if(SignExtend) {
          result.limbs[ir] = ((signed_limb_type)(a.limbs[ia0]) >> sub_limb_offset_0);
        }
        else {
          result.limbs[ir] = ((a.limbs[ia0]) >> sub_limb_offset_0);
        }
      }
      else if(ia1 < Limbs) {
        result.limbs[ir] = (a.limbs[ia1] << sub_limb_offset_1);
      }
      else if(ir < limb_offset) {
        result.limbs[ir] = 0;
      }
      else {
        result.limbs[ir] = sign_ext;
      }
    }
  }
  return result;
}
// TODO is that a good shift argument type?
template<size_t LimbsOut, size_t Limbs>
inline bignum<LimbsOut> shift_left_zero_extend(bignum<Limbs> a, uint32_t shift) {
  const size_t limb_offset = shift / limb_bits;
  const size_t sub_limb_offset_0 = shift % limb_bits;
  return shift_impl<false, LimbsOut>(a, limb_offset, sub_limb_offset_0);
}
template<size_t LimbsOut, size_t Limbs>
inline bignum<LimbsOut> shift_right_zero_extend(bignum<Limbs> a, uint32_t shift) {
  ptrdiff_t limb_offset = shift / limb_bits;
  ptrdiff_t sub_limb_offset_0 = shift % limb_bits;
  if(sub_limb_offset_0 != 0) {
    ++limb_offset;
    sub_limb_offset_0 = limb_bits - sub_limb_offset_0;
  }
  return shift_impl<false, LimbsOut>(a, -limb_offset, sub_limb_offset_0);
}
template<size_t LimbsOut, size_t Limbs>
inline bignum<LimbsOut> shift_left_sign_extend(bignum<Limbs> a, uint32_t shift) {
  const size_t limb_offset = shift / limb_bits;
  const size_t sub_limb_offset_0 = shift % limb_bits;
  return shift_impl<false, LimbsOut>(a, limb_offset, sub_limb_offset_0);
}
template<size_t LimbsOut, size_t Limbs>
inline bignum<LimbsOut> shift_right_sign_extend(bignum<Limbs> a, uint32_t shift) {
  ptrdiff_t limb_offset = shift / limb_bits;
  ptrdiff_t sub_limb_offset_0 = shift % limb_bits;
  if(sub_limb_offset_0 != 0) {
    ++limb_offset;
    sub_limb_offset_0 = limb_bits - sub_limb_offset_0;
  }
  return shift_impl<true, LimbsOut>(a, -limb_offset, sub_limb_offset_0);
}




#if 0
// problem: sign extension whetherness.
template<size_t LimbsOut, size_t LimbsA, size_t LimbsB>
inline bignum<LimbsOut> add(bignum<LimbsA> a, bignum<LimbsB> b) {
  bignum<LimbsOut> result = {{}};
  bool carry = false;
  for(size_t ir = 0; ir != LimbsOut; ++ir) {
    result.limbs[ir] = (limb_type)(carry);
    carry = false;
    if(ir < LimbsA) {
      const limb_type oldlimb = result.limbs[ir];
      const limb_type newlimb = oldlimb + a.limbs[ir];
      carry = (carry || newlimb < oldlimb);
    }
    if(ir < LimbsB) {
      const limb_type oldlimb = result.limbs[ir];
      const limb_type newlimb = oldlimb + b.limbs[ir];
      carry = (carry || newlimb < oldlimb);
    }
  }
  return result;
}
#endif
//TODO subtract, bitwise

//cast_unsigned
template<size_t LimbsOut, size_t LimbsA>
inline bignum<LimbsOut> zero_extend(bignum<LimbsA> a) {
  bignum<LimbsOut> result;
  for(size_t ir = 0; ir < LimbsA && ir < LimbsOut; ++ir) {
    result.limbs[ir] = a.limbs[ir];
  }
  for(size_t ir = LimbsA; ir < LimbsOut; ++ir) {
    result.limbs[ir] = 0;
  }
  return result;
}
template<size_t LimbsOut>
inline bignum<LimbsOut> zero_extend_from_limb(limb_type a) {
  bignum<LimbsOut> result = {{}};
  if(LimbsOut > 0) {
    result.limbs[0] = a;
  }
  return result;
}

template<size_t LimbsOut>
inline bignum<LimbsOut> zero_extend_from_uint64(uint64_t a) {
  static_assert(limb_bits >= 64, "bug");
  return zero_extend_from_limb<LimbsOut>(a);
}
template<size_t LimbsOut>
inline bignum<LimbsOut> zero_extend_from_uint32(uint32_t a) {
  static_assert(limb_bits >= 32, "bug");
  return zero_extend_from_limb<LimbsOut>(a);
}


template<size_t LimbsOut, size_t LimbsA>
inline bignum<LimbsOut> sign_extend(bignum<LimbsA> a) {
  bignum<LimbsOut> result;
  const bool neg = is_negative(a);
  const limb_type sign_ext = (neg ? (limb_type)(-1) : (limb_type)(0));
  for(size_t ir = 0; ir != LimbsA && ir != LimbsOut; ++ir) {
    result.limbs[ir] = a.limbs[ir];
  }
  if(LimbsOut > LimbsA) {
    for(size_t ir = LimbsA; ir != LimbsOut; ++ir) {
      result.limbs[ir] = sign_ext;
    }
  }
  return result;
}

template<size_t LimbsOut>
inline bignum<LimbsOut> sign_extend_from_limb(signed_limb_type a) {
  return sign_extend<LimbsOut>(bignum<1>{{(limb_type)(a)}});
}

template<size_t LimbsOut>
inline bignum<LimbsOut> sign_extend_from_int64(int64_t a) {
  static_assert(limb_bits >= 64, "bug");
  return sign_extend_from_limb<LimbsOut>((limb_type)(signed_limb_type)(a));
}
template<size_t LimbsOut>
inline bignum<LimbsOut> sign_extend_from_int32(int32_t a) {
  static_assert(limb_bits >= 32, "bug");
  return sign_extend_from_limb<LimbsOut>((limb_type)(signed_limb_type)(a));
}

template<size_t LimbsOut, size_t Limbs>
inline bignum<LimbsOut> divide_by_limb_unsigned(bignum<Limbs> a, limb_type b) {
  
}
template<size_t Limbs>
inline void show_limbs_hex_bigendian(bignum<Limbs> a, char out[Limbs*(limb_bits/4 + 1)]) {
  size_t i = 0;
  for(size_t ia = 0; ia != Limbs; ++ia) {
    limb_type val = a.limbs[Limbs-1-ia];
    //LOG <<std::hex<< "?" << val << "?";
    for(int off = limb_bits - 4; off >= 0; off -= 4) {
      const char nybble = (char)((val >> off) & 0xf);
      out[i++] = ((nybble < 0xa) ? ('0'+nybble) : ('a'+nybble-0xa));
    }
    out[i++] = ' ';
  }
  out[i-1] = '\0';
}


//bounds checking could be stuck in these fairly runtime-efficiently.
//these are fairly macro-able.

template<size_t Bits>
struct biguint : bignum<(Bits/limb_bits)> {
  static_assert(Bits % limb_bits == 0, "There is no bigint with bit-size not a multiple of limb size.");
  static const size_t bignum_limbs = Bits/limb_bits;
  typedef bignum<bignum_limbs> bignum_type;

  biguint() {}

  biguint(uint64_t i) : bignum_type(zero_extend_from_uint64<bignum_limbs>(i)) {}

  biguint(bignum_type i) : bignum_type(i) {}

  template<size_t OtherBits>
  biguint(biguint<OtherBits> i,
    typename boost::enable_if_c<(OtherBits < Bits)>::type* = 0
  ) : bignum_type(zero_extend<bignum_limbs>(i)) {}
  template<size_t OtherBits>
  explicit biguint(biguint<OtherBits> i,
    typename boost::enable_if_c<(OtherBits > Bits)>::type* = 0
  ) : bignum_type(zero_extend<bignum_limbs>(i)) {}

  explicit operator bool()const { return nonzero(*this); }
friend inline biguint<Bits> operator*(biguint<Bits> a, biguint<Bits> b)
{ return long_multiply_unsigned<biguint<Bits>::bignum_limbs>(a, b); }

friend inline biguint<Bits> operator+(biguint<Bits> a) { return a; }
friend inline biguint<Bits> operator-(biguint<Bits> a) { return negate(a); }
friend inline biguint<Bits> operator+(biguint<Bits> a, biguint<Bits> b) { return add(a, b); }
friend inline biguint<Bits> operator-(biguint<Bits> a, biguint<Bits> b) { return subtract(a, b); }

friend inline biguint<Bits> operator&(biguint<Bits> a, biguint<Bits> b) { return bitwise_and(a, b); }
friend inline biguint<Bits> operator|(biguint<Bits> a, biguint<Bits> b) { return bitwise_or(a, b); }
friend inline biguint<Bits> operator^(biguint<Bits> a, biguint<Bits> b) { return bitwise_xor(a, b); }
friend inline biguint<Bits> operator~(biguint<Bits> a) { return bitwise_complement(a); }

friend inline biguint<Bits> operator<<(biguint<Bits> a, uint32_t shift) { return shift_left_zero_extend<biguint<Bits>::bignum_limbs>(a, shift); }
friend inline biguint<Bits> operator>>(biguint<Bits> a, uint32_t shift) { return shift_right_zero_extend<biguint<Bits>::bignum_limbs>(a, shift); }

friend inline bool operator==(biguint<Bits> a, biguint<Bits> b) { return equal(a, b); }
friend inline bool operator!=(biguint<Bits> a, biguint<Bits> b) { return !equal(a, b); }
friend inline bool operator<(biguint<Bits> a, biguint<Bits> b) { return less_than_unsigned(a, b); }
friend inline bool operator>(biguint<Bits> a, biguint<Bits> b) { return less_than_unsigned(b, a); }
friend inline bool operator>=(biguint<Bits> a, biguint<Bits> b) { return !less_than_unsigned(a, b); }
friend inline bool operator<=(biguint<Bits> a, biguint<Bits> b) { return !less_than_unsigned(b, a); }


friend inline biguint<Bits>& operator+=(biguint<Bits>& a, biguint<Bits> b) { a = a + b; return a; }
friend inline biguint<Bits>& operator-=(biguint<Bits>& a, biguint<Bits> b) { a = a - b; return a; }
friend inline biguint<Bits>& operator*=(biguint<Bits>& a, biguint<Bits> b) { a = a * b; return a; }
friend inline biguint<Bits>& operator&=(biguint<Bits>& a, biguint<Bits> b) { a = a & b; return a; }
friend inline biguint<Bits>& operator|=(biguint<Bits>& a, biguint<Bits> b) { a = a | b; return a; }
friend inline biguint<Bits>& operator^=(biguint<Bits>& a, biguint<Bits> b) { a = a ^ b; return a; }
friend inline biguint<Bits>& operator<<=(biguint<Bits>& a, uint32_t shift) { a = a << shift; return a; }
friend inline biguint<Bits>& operator>>=(biguint<Bits>& a, uint32_t shift) { a = a >> shift; return a; }

friend std::ostream& operator<<(std::ostream& os, biguint<Bits> a) { return os; }
};



template<size_t Bits>
struct bigint : bignum<(Bits/limb_bits)> {
  static_assert(Bits % limb_bits == 0, "There is no bigint with bit-size not a multiple of limb size.");
  static const size_t bignum_limbs = Bits/limb_bits;
  typedef bignum<bignum_limbs> bignum_type;

  bigint() {}

  bigint(int64_t i) : bignum_type(sign_extend_from_int64<bignum_limbs>(i)) {}
  
  bigint(bignum_type i) : bignum_type(i) {}

  template<size_t OtherBits>
  bigint(bigint<OtherBits> i,
    typename boost::enable_if_c<(OtherBits < Bits)>::type* = 0
  ) : bignum_type(sign_extend<bignum_limbs>(i)) {}
  template<size_t OtherBits>
  explicit bigint(bigint<OtherBits> i,
    typename boost::enable_if_c<(OtherBits > Bits)>::type* = 0
  ) : bignum_type(sign_extend<bignum_limbs>(i)) {}

  explicit operator bool()const { return nonzero(*this); }

friend inline bigint<Bits> operator*(bigint<Bits> a, bigint<Bits> b)
{ return long_multiply_signed<biguint<Bits>::bignum_limbs>(a, b); }
friend inline bigint<Bits> operator+(bigint<Bits> a) { return a; }
friend inline bigint<Bits> operator-(bigint<Bits> a) { return negate(a); }
friend inline bigint<Bits> operator+(bigint<Bits> a, bigint<Bits> b) { return add(a, b); }
friend inline bigint<Bits> operator-(bigint<Bits> a, bigint<Bits> b) { return subtract(a, b); }

friend inline bigint<Bits> operator&(bigint<Bits> a, bigint<Bits> b) { return bitwise_and(a, b); }
friend inline bigint<Bits> operator|(bigint<Bits> a, bigint<Bits> b) { return bitwise_or(a, b); }
friend inline bigint<Bits> operator^(bigint<Bits> a, bigint<Bits> b) { return bitwise_xor(a, b); }
friend inline bigint<Bits> operator~(bigint<Bits> a) { return bitwise_complement(a); }

friend inline bool operator==(bigint<Bits> a, bigint<Bits> b) { return equal(a, b); }
friend inline bool operator!=(bigint<Bits> a, bigint<Bits> b) { return !equal(a, b); }
friend inline bool operator<(bigint<Bits> a, bigint<Bits> b) { return less_than_signed(a, b); }
friend inline bool operator>(bigint<Bits> a, bigint<Bits> b) { return less_than_signed(b, a); }
friend inline bool operator>=(bigint<Bits> a, bigint<Bits> b) { return !less_than_signed(a, b); }
friend inline bool operator<=(bigint<Bits> a, bigint<Bits> b) { return !less_than_signed(b, a); }

friend inline bigint<Bits> operator<<(bigint<Bits> a, uint32_t shift) { return shift_left_sign_extend<biguint<Bits>::bignum_limbs>(a, shift); }
friend inline bigint<Bits> operator>>(bigint<Bits> a, uint32_t shift) { return shift_right_sign_extend<biguint<Bits>::bignum_limbs>(a, shift); }

friend inline bigint<Bits>& operator+=(bigint<Bits>& a, bigint<Bits> b) { a = a + b; return a; }
friend inline bigint<Bits>& operator-=(bigint<Bits>& a, bigint<Bits> b) { a = a - b; return a; }
friend inline bigint<Bits>& operator*=(bigint<Bits>& a, bigint<Bits> b) { a = a * b; return a; }
friend inline bigint<Bits>& operator&=(bigint<Bits>& a, bigint<Bits> b) { a = a & b; return a; }
friend inline bigint<Bits>& operator|=(bigint<Bits>& a, bigint<Bits> b) { a = a | b; return a; }
friend inline bigint<Bits>& operator^=(bigint<Bits>& a, bigint<Bits> b) { a = a ^ b; return a; }
friend inline bigint<Bits>& operator<<=(bigint<Bits>& a, uint32_t shift) { a = a << shift; return a; }
friend inline bigint<Bits>& operator>>=(bigint<Bits>& a, uint32_t shift) { a = a >> shift; return a; }
friend std::ostream& operator<<(std::ostream& os, bigint<Bits> a) {
  char out[biguint<Bits>::bignum_limbs*(limb_bits/4 + 1)];
  show_limbs_hex_bigendian(a, out);
  os << out;
  return os;
}
};

// Or templatize on both bits and take the max?
// Might be faster depending on inlining, but requires mixed operators with non-bigint
// types to be explicitly defined.
/*
template<size_t Bits> inline biguint<Bits> operator*(biguint<Bits> a, biguint<Bits> b)
{ return long_multiply_unsigned<biguint<Bits>::bignum_limbs>(a, b); }
template<size_t Bits> inline bigint<Bits> operator*(bigint<Bits> a, bigint<Bits> b)
{ return long_multiply_signed<biguint<Bits>::bignum_limbs>(a, b); }
*/
// For explicit multiplication-precision control:
template<size_t Bits, size_t BitsA, size_t BitsB> inline biguint<Bits>
multiply_to(biguint<BitsA> a, biguint<BitsB> b) { return long_multiply_unsigned<biguint<Bits>::bignum_limbs>(a, b); }
template<size_t Bits, size_t BitsA, size_t BitsB> inline bigint<Bits>
multiply_to(bigint<BitsA> a, bigint<BitsB> b) { return long_multiply_signed<biguint<Bits>::bignum_limbs>(a, b); }
template<size_t BitsA, size_t BitsB> inline biguint<(BitsA+BitsB)>
multiply_to_fit(biguint<BitsA> a, biguint<BitsB> b) { return long_multiply_unsigned<biguint<(BitsA+BitsB)>::bignum_limbs>(a, b); }
template<size_t BitsA, size_t BitsB> inline bigint<(BitsA+BitsB)>
multiply_to_fit(bigint<BitsA> a, bigint<BitsB> b) { return long_multiply_signed<biguint<(BitsA+BitsB)>::bignum_limbs>(a, b); }

#if 0
// gcc explorer tests
#include <stdint.h>
#include <stdlib.h>
using namespace std;
typedef __uint128_t DETECTED_uint128_t;

extern void g(bignum<6> const&);
extern int q,p,r,s,t,u,v,w;
void f() { g(long_multiply<6>(bignum<4>{q,p,t,u},bignum<4>{r,s,v,w})); }
#endif

#endif

