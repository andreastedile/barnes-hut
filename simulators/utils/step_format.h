#ifndef BARNES_HUT_STEP_FORMAT_H
#define BARNES_HUT_STEP_FORMAT_H

#include <iomanip>
#include <sstream>
#include <string>

namespace bh {

std::string format_step_n(int step_n, int n_steps) {
  int n_chars = static_cast<int>(std::to_string(n_steps).length());
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(n_chars) << step_n;
  return ss.str();
}

}  // namespace bh

#endif  // BARNES_HUT_STEP_FORMAT_H
