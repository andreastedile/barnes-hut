#ifndef BARNES_HUT_POWER_OF_FOUR_H
#define BARNES_HUT_POWER_OF_FOUR_H

#include <cmath>

bool is_power_of_four(int n) {
  auto log_base_four = std::log(n) / std::log(4);
  return log_base_four - std::floor(log_base_four) == 0;
}

#endif  // BARNES_HUT_POWER_OF_FOUR_H
