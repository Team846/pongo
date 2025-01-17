#include "frc846/math/DoubleSyncBuffer.h"

#include <iostream>
#include <stdexcept>

namespace frc846::math {

DoubleSyncBuffer::DoubleSyncBuffer(size_t sz, int max_sync_diff)
    : def_size{sz}, max_sync_diff_{max_sync_diff} {
  m_buffer_1.reserve(def_size);
  m_buffer_2.reserve(def_size);
}

void DoubleSyncBuffer::Add(double val1, double val2) {
  m_buffer_1.push_back(val1);
  m_buffer_2.push_back(val2);

  if (m_buffer_1.size() > def_size) {
    m_buffer_1.erase(m_buffer_1.begin());
    m_buffer_2.erase(m_buffer_2.begin());
  }
}

bool DoubleSyncBuffer::IsValid() {
  if (m_buffer_1.size() != m_buffer_2.size())
    throw std::runtime_error(
        "DoubleSyncBuffer contained buffer sizes are inequal");

  return m_buffer_1.size() == def_size;
}

void DoubleSyncBuffer::Sync() {
  if (!IsValid())
    throw std::runtime_error(
        "DoubleSyncBuffer::Sync() called on invalid buffer");

  sync_diff_ = 0;

  double max_correlation = -1.0;

  for (int shift = 0; shift < max_sync_diff_; shift++) {
    double correlation = 0.0;
    size_t sz = m_buffer_2.size() - shift;
    for (size_t i = 0; i < sz; i++) {
      correlation += m_buffer_1[i] * m_buffer_2[i + shift];
    }
    correlation /= sz;
    if (correlation > max_correlation) {
      max_correlation = correlation;
      sync_diff_ = shift;
    }
  }
}

std::pair<double, double> DoubleSyncBuffer::GetTrough() {
  if (!IsValid())
    throw std::runtime_error(
        "DoubleSyncBuffer::GetTrough() called on invalid buffer");

  double min_combination = 100000000.0;
  double min1 = 100000000.0;
  double min2 = 100000000.0;

  for (size_t i = 1; i < m_buffer_2.size() - sync_diff_; i++) {
    double added = m_buffer_2[i + sync_diff_];
    if (added < min_combination) {
      min_combination = added;
      min1 = m_buffer_1[i];
      min2 = m_buffer_2[i + sync_diff_];
    }
  }

  return std::make_pair(min1, min2);
}

}  // namespace frc846::math