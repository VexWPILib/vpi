// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

/*
 * This code is a modified version of Benjamin Jurke's work in 2015. You can read his blog post
 * here:
 * https://benjaminjurke.com/content/articles/2015/compile-time-numerical-unit-dimension-checking/
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <cmath>
#include <ratio>

namespace vpi {
template <typename MassDim, typename LengthDim, typename TimeDim, typename AngleDim, typename CoulombDim>
class RQuantity {
  public:
  explicit constexpr RQuantity() : value(0.0) {
  }

  explicit constexpr RQuantity(double val) : value(val) {
  }

  explicit constexpr RQuantity(long double val) : value(static_cast<double>(val)) {
  }

  // The intrinsic operations for a quantity with a unit is addition and subtraction
  const RQuantity &operator+=(const RQuantity &rhs) {
    value += rhs.value;
    return *this;
  }

  const RQuantity &operator-=(const RQuantity &rhs) {
    value -= rhs.value;
    return *this;
  }

  const RQuantity operator-() {
    return RQuantity(value * -1);
  }

  const RQuantity &operator*=(const double rhs) {
    value *= rhs;
    return *this;
  }

  const RQuantity &operator/=(const double rhs) {
    value /= rhs;
    return *this;
  }

  // Returns the value of the quantity in multiples of the specified unit
  constexpr double convert(const RQuantity &rhs) const {
    return value / rhs.value;
  }

  // returns the raw value of the quantity (should not be used)
  constexpr double getValue() const {
    return value;
  }

  constexpr RQuantity<MassDim, LengthDim, TimeDim, AngleDim, CoulombDim> abs() const {
    return RQuantity<MassDim, LengthDim, TimeDim, AngleDim, CoulombDim>(std::fabs(value));
  }

  constexpr RQuantity<std::ratio_divide<MassDim, std::ratio<2>>,
                      std::ratio_divide<LengthDim, std::ratio<2>>,
                      std::ratio_divide<TimeDim, std::ratio<2>>,
                      std::ratio_divide<AngleDim, std::ratio<2>>,
                      std::ratio_divide<CoulombDim, std::ratio<2>>>
  sqrt() const {
    return RQuantity<std::ratio_divide<MassDim, std::ratio<2>>,
                     std::ratio_divide<LengthDim, std::ratio<2>>,
                     std::ratio_divide<TimeDim, std::ratio<2>>,
                     std::ratio_divide<AngleDim, std::ratio<2>>,
                     std::ratio_divide<CoulombDim, std::ratio<2>>>(std::sqrt(value));
  }

  private:
  double value;
};

// Predefined (physical unit) quantity types:
// ------------------------------------------
#define QUANTITY_TYPE(_Mdim, _Ldim, _Tdim, _Adim, _Cdim, name)                                            \
  typedef RQuantity<std::ratio<_Mdim>, std::ratio<_Ldim>, std::ratio<_Tdim>, std::ratio<_Adim>, std::ratio<_Cdim>>    \
    name;

// Unitless
QUANTITY_TYPE(0, 0, 0, 0, 0, Number)
constexpr Number number(1.0);

// Standard arithmetic operators:
// ------------------------------
template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> operator+(const RQuantity<M, L, T, A, C> &lhs,
                                          const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<M, L, T, A, C>(lhs.getValue() + rhs.getValue());
}
template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> operator-(const RQuantity<M, L, T, A, C> &lhs,
                                          const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<M, L, T, A, C>(lhs.getValue() - rhs.getValue());
}
template <typename M1,
          typename L1,
          typename T1,
          typename A1,
          typename C1,
          typename M2,
          typename L2,
          typename T2,
          typename A2,
          typename C2>
constexpr RQuantity<std::ratio_add<M1, M2>,
                    std::ratio_add<L1, L2>,
                    std::ratio_add<T1, T2>,
                    std::ratio_add<A1, A2>,
                    std::ratio_add<C1, C2>>
operator*(const RQuantity<M1, L1, T1, A1, C1> &lhs, const RQuantity<M2, L2, T2, A2, C2> &rhs) {
  return RQuantity<std::ratio_add<M1, M2>,
                   std::ratio_add<L1, L2>,
                   std::ratio_add<T1, T2>,
                   std::ratio_add<A1, A2>,
                   std::ratio_add<C1, C2>>(lhs.getValue() * rhs.getValue());
}
template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> operator*(const double &lhs, const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<M, L, T, A, C>(lhs * rhs.getValue());
}
template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> operator*(const RQuantity<M, L, T, A, C> &lhs, const double &rhs) {
  return RQuantity<M, L, T, A, C>(lhs.getValue() * rhs);
}
template <typename M1,
          typename L1,
          typename T1,
          typename A1,
          typename C1,
          typename M2,
          typename L2,
          typename T2,
          typename A2,
          typename C2>
constexpr RQuantity<std::ratio_subtract<M1, M2>,
                    std::ratio_subtract<L1, L2>,
                    std::ratio_subtract<T1, T2>,
                    std::ratio_subtract<A1, A2>,
                    std::ratio_subtract<C1, C2>>
operator/(const RQuantity<M1, L1, T1, A1, C1> &lhs, const RQuantity<M2, L2, T2, A2, C2> &rhs) {
  return RQuantity<std::ratio_subtract<M1, M2>,
                   std::ratio_subtract<L1, L2>,
                   std::ratio_subtract<T1, T2>,
                   std::ratio_subtract<A1, A2>,
                   std::ratio_subtract<C1, C2>>(lhs.getValue() / rhs.getValue());
}
template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<std::ratio_subtract<std::ratio<0>, M>,
                    std::ratio_subtract<std::ratio<0>, L>,
                    std::ratio_subtract<std::ratio<0>, T>,
                    std::ratio_subtract<std::ratio<0>, A>,
                    std::ratio_subtract<std::ratio<0>, C>>
operator/(const double &x, const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<std::ratio_subtract<std::ratio<0>, M>,
                   std::ratio_subtract<std::ratio<0>, L>,
                   std::ratio_subtract<std::ratio<0>, T>,
                   std::ratio_subtract<std::ratio<0>, A>,
                   std::ratio_subtract<std::ratio<0>, C>>(x / rhs.getValue());
}
template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> operator/(const RQuantity<M, L, T, A, C> &rhs, const double &x) {
  return RQuantity<M, L, T, A, C>(rhs.getValue() / x);
}

// Comparison operators for quantities:
// ------------------------------------
template <typename M, typename L, typename T, typename A, typename C>
constexpr bool operator==(const RQuantity<M, L, T, A, C> &lhs, const RQuantity<M, L, T, A, C> &rhs) {
  return (lhs.getValue() == rhs.getValue());
}
template <typename M, typename L, typename T, typename A, typename C>
constexpr bool operator!=(const RQuantity<M, L, T, A, C> &lhs, const RQuantity<M, L, T, A, C> &rhs) {
  return (lhs.getValue() != rhs.getValue());
}
template <typename M, typename L, typename T, typename A, typename C>
constexpr bool operator<=(const RQuantity<M, L, T, A, C> &lhs, const RQuantity<M, L, T, A, C> &rhs) {
  return (lhs.getValue() <= rhs.getValue());
}
template <typename M, typename L, typename T, typename A, typename C>
constexpr bool operator>=(const RQuantity<M, L, T, A, C> &lhs, const RQuantity<M, L, T, A, C> &rhs) {
  return (lhs.getValue() >= rhs.getValue());
}
template <typename M, typename L, typename T, typename A, typename C>
constexpr bool operator<(const RQuantity<M, L, T, A, C> &lhs, const RQuantity<M, L, T, A, C> &rhs) {
  return (lhs.getValue() < rhs.getValue());
}
template <typename M, typename L, typename T, typename A, typename C>
constexpr bool operator>(const RQuantity<M, L, T, A, C> &lhs, const RQuantity<M, L, T, A, C> &rhs) {
  return (lhs.getValue() > rhs.getValue());
}

// Common math functions:
// ------------------------------
template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> abs(const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<M, L, T, A, C>(std::abs(rhs.getValue()));
}

template <typename R, typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<std::ratio_multiply<M, R>,
                    std::ratio_multiply<L, R>,
                    std::ratio_multiply<T, R>,
                    std::ratio_multiply<A, R>,
                    std::ratio_multiply<C, R>>
pow(const RQuantity<M, L, T, A, C> &lhs) {
  return RQuantity<std::ratio_multiply<M, R>,
                   std::ratio_multiply<L, R>,
                   std::ratio_multiply<T, R>,
                   std::ratio_multiply<A, R>,
                   std::ratio_multiply<C, R>>(std::pow(lhs.getValue(), double(R::num) / R::den));
}

template <int R, typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<std::ratio_multiply<M, std::ratio<R>>,
                    std::ratio_multiply<L, std::ratio<R>>,
                    std::ratio_multiply<T, std::ratio<R>>,
                    std::ratio_multiply<A, std::ratio<R>>,
                    std::ratio_multiply<C, std::ratio<R>>>
pow(const RQuantity<M, L, T, A, C> &lhs) {
  return RQuantity<std::ratio_multiply<M, std::ratio<R>>,
                   std::ratio_multiply<L, std::ratio<R>>,
                   std::ratio_multiply<T, std::ratio<R>>,
                   std::ratio_multiply<A, std::ratio<R>>,
                   std::ratio_multiply<C, std::ratio<R>>>(std::pow(lhs.getValue(), R));
}

template <int R, typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<std::ratio_divide<M, std::ratio<R>>,
                    std::ratio_divide<L, std::ratio<R>>,
                    std::ratio_divide<T, std::ratio<R>>,
                    std::ratio_divide<A, std::ratio<R>>,
                    std::ratio_divide<C, std::ratio<R>>>
root(const RQuantity<M, L, T, A, C> &lhs) {
  return RQuantity<std::ratio_divide<M, std::ratio<R>>,
                   std::ratio_divide<L, std::ratio<R>>,
                   std::ratio_divide<T, std::ratio<R>>,
                   std::ratio_divide<A, std::ratio<R>>,
                   std::ratio_divide<C, std::ratio<R>>>(std::pow(lhs.getValue(), 1.0 / R));
}

template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<std::ratio_divide<M, std::ratio<2>>,
                    std::ratio_divide<L, std::ratio<2>>,
                    std::ratio_divide<T, std::ratio<2>>,
                    std::ratio_divide<A, std::ratio<2>>,
                    std::ratio_divide<C, std::ratio<2>>>
sqrt(const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<std::ratio_divide<M, std::ratio<2>>,
                   std::ratio_divide<L, std::ratio<2>>,
                   std::ratio_divide<T, std::ratio<2>>,
                   std::ratio_divide<A, std::ratio<2>>,
                   std::ratio_divide<C, std::ratio<2>>>(std::sqrt(rhs.getValue()));
}

template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<std::ratio_divide<M, std::ratio<3>>,
                    std::ratio_divide<L, std::ratio<3>>,
                    std::ratio_divide<T, std::ratio<3>>,
                    std::ratio_divide<A, std::ratio<3>>,
                    std::ratio_divide<C, std::ratio<3>>>
cbrt(const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<std::ratio_divide<M, std::ratio<3>>,
                   std::ratio_divide<L, std::ratio<3>>,
                   std::ratio_divide<T, std::ratio<3>>,
                   std::ratio_divide<A, std::ratio<3>>,
                   std::ratio_divide<C, std::ratio<3>>>(std::cbrt(rhs.getValue()));
}

template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<std::ratio_multiply<M, std::ratio<2>>,
                    std::ratio_multiply<L, std::ratio<2>>,
                    std::ratio_multiply<T, std::ratio<2>>,
                    std::ratio_multiply<A, std::ratio<2>>,
                    std::ratio_multiply<C, std::ratio<2>>>
square(const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<std::ratio_multiply<M, std::ratio<2>>,
                   std::ratio_multiply<L, std::ratio<2>>,
                   std::ratio_multiply<T, std::ratio<2>>,
                   std::ratio_multiply<A, std::ratio<2>>,
                   std::ratio_multiply<C, std::ratio<2>>>(std::pow(rhs.getValue(), 2));
}

template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<std::ratio_multiply<M, std::ratio<3>>,
                    std::ratio_multiply<L, std::ratio<3>>,
                    std::ratio_multiply<T, std::ratio<3>>,
                    std::ratio_multiply<A, std::ratio<3>>,
                    std::ratio_multiply<C, std::ratio<3>>>
cube(const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<std::ratio_multiply<M, std::ratio<3>>,
                   std::ratio_multiply<L, std::ratio<3>>,
                   std::ratio_multiply<T, std::ratio<3>>,
                   std::ratio_multiply<A, std::ratio<3>>,
                   std::ratio_multiply<C, std::ratio<3>>>(std::pow(rhs.getValue(), 3));
}

template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> hypot(const RQuantity<M, L, T, A, C> &lhs,
                                      const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<M, L, T, A, C>(std::hypot(lhs.getValue(), rhs.getValue()));
}

template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> mod(const RQuantity<M, L, T, A, C> &lhs,
                                    const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<M, L, T, A, C>(std::fmod(lhs.getValue(), rhs.getValue()));
}

template <typename M1,
          typename L1,
          typename T1,
          typename A1,
          typename C1,
          typename M2,
          typename L2,
          typename T2,
          typename A2,
          typename C2>
constexpr RQuantity<M1, L1, T1, A1, C1> copysign(const RQuantity<M1, L1, T1, A1, C1> &lhs,
                                             const RQuantity<M2, L2, T2, A2, C2> &rhs) {
  return RQuantity<M1, L1, T1, A1, C1>(std::copysign(lhs.getValue(), rhs.getValue()));
}

template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> ceil(const RQuantity<M, L, T, A, C> &lhs,
                                     const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<M, L, T, A, C>(std::ceil(lhs.getValue() / rhs.getValue()) * rhs.getValue());
}

template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> floor(const RQuantity<M, L, T, A, C> &lhs,
                                      const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<M, L, T, A, C>(std::floor(lhs.getValue() / rhs.getValue()) * rhs.getValue());
}

template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> trunc(const RQuantity<M, L, T, A, C> &lhs,
                                      const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<M, L, T, A, C>(std::trunc(lhs.getValue() / rhs.getValue()) * rhs.getValue());
}

template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<M, L, T, A, C> round(const RQuantity<M, L, T, A, C> &lhs,
                                      const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<M, L, T, A, C>(std::round(lhs.getValue() / rhs.getValue()) * rhs.getValue());
}

// Common trig functions:
// ------------------------------
/*
const Number
sin(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>> &rhs) {
  return Number(std::sin(rhs.getValue()));
}

const Number
cos(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>> &rhs) {
  return Number(std::cos(rhs.getValue()));
}

const Number
tan(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>> &rhs) {
  return Number(std::tan(rhs.getValue()));
}

const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>
asin(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>(
    std::asin(rhs.getValue()));
}

const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>
acos(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>(
    std::acos(rhs.getValue()));
}

const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>
atan(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>(
    std::atan(rhs.getValue()));
}

const Number
sinh(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>> &rhs) {
  return Number(std::sinh(rhs.getValue()));
}

const Number
cosh(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>> &rhs) {
  return Number(std::cosh(rhs.getValue()));
}

const Number
tanh(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>> &rhs) {
  return Number(std::tanh(rhs.getValue()));
}

const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>
asinh(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>(
    std::asinh(rhs.getValue()));
}

const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>
acosh(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>(
    std::acosh(rhs.getValue()));
}

const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>
atanh(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>(
    std::atanh(rhs.getValue()));
}

template <typename M, typename L, typename T, typename A, typename C>
constexpr RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>>
atan2(const RQuantity<M, L, T, A, C> &lhs, const RQuantity<M, L, T, A, C> &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>,std::ratio<0>>(
    std::atan2(lhs.getValue(), rhs.getValue()));
}
*/
inline namespace literals {
constexpr long double operator"" _pi(long double x) {
  return static_cast<double>(x) * 3.1415926535897932384626433832795;
}
constexpr long double operator"" _pi(unsigned long long int x) {
  return static_cast<double>(x) * 3.1415926535897932384626433832795;
}
} // namespace literals
} // namespace vpi

// Conversion macro, which utilizes the string literals
#define ConvertTo(_x, _y) (_x).convert(1.0_##_y)
