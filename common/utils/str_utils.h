/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include <string>

namespace common {

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
  std::ostringstream out;
  out.precision(n);
  out << std::fixed << a_value;
  return out.str();
}

}  // namespace common
