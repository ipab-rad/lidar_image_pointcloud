#pragma once

#include <chrono>

class SimpleTimer
{
public:
  SimpleTimer() = default;

  void tick() { start_ = std::chrono::high_resolution_clock::now(); }

  double tock() const
  {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start_);

    // Return duration in miliseoncds
    // return static_cast<double>(duration_us.count()) / 1000.0;
    // return static_cast<double>(duration_us.count());
    return static_cast<double>(duration_ns.count()) / 1'000'000.0;
  }

private:
  std::chrono::high_resolution_clock::time_point start_;
};
