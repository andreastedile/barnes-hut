#ifndef BARNES_HUT_PERSISTENCE_H
#define BARNES_HUT_PERSISTENCE_H

#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

namespace bh {

template <typename StepType>
void write_to_file(const StepType& step, const std::string& filename) {
  nlohmann::json j = step;
  std::ofstream o(filename);
  o << j;
  o.close();
}

}  // namespace bh

#endif  // BARNES_HUT_PERSISTENCE_H
