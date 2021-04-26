#include <vector>

#include "gp_planner/gp/utils/bounded_penalty_function.h"
#include "gp_planner/thirdparty/matplotlib-cpp/matplotlibcpp.h"

int main(int argc, char const *argv[]) {
  planning::BoundedPenaltyFunction f(5, 2);

  std::vector<double> t, val, der;
  double tmp;
  for (double x = -10; x < 10.1; x += 0.1) {
    t.emplace_back(x);
    val.emplace_back(f.GetPenaltyAndGradient(x, &tmp));
    der.emplace_back(tmp);
  }

  namespace plt = matplotlibcpp;
  plt::plot(t,val);
  plt::figure();
  plt::plot(t, der);
  plt::show();

  return 0;
}
