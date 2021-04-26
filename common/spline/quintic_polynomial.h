/* Copyright 2019 Unity-Drive Inc. All rights reserved */

#pragma once

#include <array>

namespace common {

using QuinticCoefficients = std::array<double, 6>;

/**
 * @brief Fifth order polynomial
 *
 * Parameterization:
 * f(s) = a0 +a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5.
 */
class QuinticPolynomial {
 public:
  explicit QuinticPolynomial() = default;
  explicit QuinticPolynomial(std::array<double, 6> coeff) : coeff_(coeff) {}

  inline double f(const double s) const;
  inline double df(const double s) const;
  inline double ddf(const double s) const;
  inline double dddf(const double s) const;

  inline double operator()(const double s) const { return f(s); }
  inline void set_coeff(const QuinticCoefficients& coeff);
  inline const QuinticCoefficients& coeff() const { return coeff_; }

  inline void print() const;

 private:
  QuinticCoefficients coeff_;

  static constexpr std::array<double, 3> ddf_coef_{2.0, 6.0, 12.0};
  static constexpr std::array<double, 2> dddf_coef_{6.0, 24.0};
};

// inline function
inline void QuinticPolynomial::set_coeff(const std::array<double, 6>& coeff) {
  coeff_ = coeff;
}

inline double QuinticPolynomial::f(const double s) const {
  double p = coeff_[5];
  for (int i = 4; i >= 0; --i) {
    p = s * p + coeff_[i];
  }
  return p;
}

inline double QuinticPolynomial::df(const double s) const {
  double p = coeff_[5] * 5.0;
  for (int i = 4; i >= 1; --i) {
    p = (s * p + coeff_[i] * static_cast<double>(i));
  }
  return p;
}

inline double QuinticPolynomial::ddf(const double s) const {
  double p = coeff_[5] * 20.0;
  for (int i = 4; i >= 2; --i) {
    p = (s * p + coeff_[i] * ddf_coef_[i - 2]);
  }
  return p;
}

inline double QuinticPolynomial::dddf(const double s) const {
  double p = coeff_[5] * 60.0;
  for (int i = 4; i >= 3; --i) {
    p = (s * p + coeff_[i] * dddf_coef_[i - 3]);
  }
  return p;
}

inline void QuinticPolynomial::print() const {
  printf("[%f,%f,%f,%f,%f,%f]\n", coeff_[0], coeff_[1], coeff_[2], coeff_[3],
         coeff_[4], coeff_[5]);
}
}  // namespace common