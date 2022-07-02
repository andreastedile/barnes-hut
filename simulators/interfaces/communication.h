#ifndef BARNES_HUT_COMMUNICATION_H
#define BARNES_HUT_COMMUNICATION_H

namespace bh {

class ICommunication {
 public:
  ICommunication(int proc_id, int n_procs);

  const int proc_id;
  const int n_procs;
};

}  // namespace bh

#endif  // BARNES_HUT_COMMUNICATION_H
