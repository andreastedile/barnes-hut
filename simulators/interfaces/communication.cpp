#include "communication.h"

namespace bh {

ICommunication::ICommunication(int proc_id, int n_procs)
    : proc_id{proc_id}, n_procs{n_procs} {}

}  // namespace bh
