#ifndef BARNES_HUT_BODY_DESERIALIZATION_H
#define BARNES_HUT_BODY_DESERIALIZATION_H

#include <vector>

#include "body.h"
#include "mpi_datatypes.h"

namespace bh {

Body deserialize_body(const mpi::Body &body);

std::vector<Body> deserialize_bodies(const std::vector<mpi::Body> &bodies);

}  // namespace bh

#endif  // BARNES_HUT_BODY_DESERIALIZATION_H
