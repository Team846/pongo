#pragma once

#include <stddef.h>

#include <vector>

namespace frc846::math {

/*
DoubleSyncBuffer

A class that contains two rolling buffers of doubles and syncs them.
*/
class DoubleSyncBuffer {
public:
  /*
  DoubleSyncBuffer()

  Constructs a DoubleSyncBuffer with the given size.
  */
  DoubleSyncBuffer(size_t sz = 50U, int max_sync_diff = 15);

  /*
  Add()

  Adds a value to each contained buffer. The values may be unsynced. Sync() will
  be called automatically.

  The second signal may be time delayed.
  */
  void Add(double val1, double val2);

  bool IsValid();

  // Returns the computed sync difference between each container buffer
  int GetSyncDiff() { return sync_diff_; }

  std::pair<double, double> GetTrough();

private:
  // Computes the sync difference between the two buffers
  void Sync();

  std::vector<double> m_buffer_1;
  std::vector<double> m_buffer_2;

  size_t def_size;
  int max_sync_diff_;

  int sync_diff_;
};

}  // namespace frc846::math