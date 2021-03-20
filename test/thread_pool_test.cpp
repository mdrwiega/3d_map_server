#include <unordered_map>
#include <unordered_set>

#include <thread_pool.h>
#include "gtest/gtest.h"

TEST(ThreadPoolTest_, CreateThreadPoolError) {
  try {
    auto thread_pool = thread_pool::ThreadPool(0);
  } catch (std::invalid_argument& exception) {
    EXPECT_STREQ(exception.what(), "Wrong number of threads.");
  }
}

TEST(ThreadPoolTest, ParallelCalculation) {
  thread_pool::ThreadPool thread_pool;

  std::vector<std::vector<std::uint32_t>> data(10);
  for (auto& it: data) {
    it.reserve(100000);
    for (std::uint32_t i = 0; i < 100000; ++i) {
      it.push_back(i);
    }
  }

  auto do_some_calculation = [](std::vector<std::uint32_t>& src) {
    for (std::uint32_t i = 0; i < src.size() - 1; ++i) {
        src[i] = (src[i] * src[i + 1]) / (src[i] - src[i + 1] * 3);
    }
  };

  std::vector<std::future<void>> thread_futures;
  for (std::uint32_t i = 0; i < data.size(); ++i) {
    thread_futures.emplace_back(thread_pool.Submit(do_some_calculation, std::ref(data[i])));
  }

  for (const auto& it: thread_futures) {
    it.wait();
  }
}