#ifndef BARNES_HUT_BARNES_HUT_H
#define BARNES_HUT_BARNES_HUT_H

namespace bh {

class IBarnesHut {
 public:
  explicit IBarnesHut(double theta);
  const double theta;
};

}  // namespace bh

#endif  // BARNES_HUT_BARNES_HUT_H
