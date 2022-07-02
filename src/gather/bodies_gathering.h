#ifndef BARNES_HUT_BODIES_GATHERING_H
#define BARNES_HUT_BODIES_GATHERING_H

#include <vector>
#include "body.h"

namespace bh {

std::vector<Body> gather_bodies(int proc_id, int n_procs, int total_n_bodies, const std::vector<Body>& my_bodies);

}

#endif  // BARNES_HUT_BODIES_GATHERING_H
