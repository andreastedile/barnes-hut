#ifndef BARNES_HUT_PHYSICS_H
#define BARNES_HUT_PHYSICS_H

namespace bh {

class IPhysics {
 public:
  explicit IPhysics(double G);
  const double G;
};

}  // namespace bh

#endif  // BARNES_HUT_PHYSICS_H
