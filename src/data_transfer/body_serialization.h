#ifndef BARNES_HUT_BODY_SERIALIZATION_H
#define BARNES_HUT_BODY_SERIALIZATION_H

#include <vector>

#include "body.h"
#include "mpi_datatypes.h"

namespace bh {

mpi::Body serialize_body(const Body& body);

std::vector<mpi::Body> serialize_bodies(const std::vector<Body>& bodies);

}  // namespace bh

#endif  // BARNES_HUT_BODY_SERIALIZATION_H
