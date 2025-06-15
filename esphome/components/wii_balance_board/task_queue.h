#pragma once

#include <functional>
#include <algorithm>

namespace esphome::wii_balance_board::detail {

class TaskQueue {
  using Task = std::tuple<uint64_t, int, std::function<void(int)>>;
  std::vector<Task> tasks;

 public:
  void add(int taskId, uint64_t when, std::function<void(int)> callable) {
    Task task{when, taskId, std::move(callable)};
    tasks.emplace_back(std::forward<Task>(std::move(task)));
    std::push_heap(tasks.begin(), tasks.end(),
                   [](const Task &a, const Task &b) { return std::get<0>(a) < std::get<0>(b); });
  }

  void process(uint64_t now) {
    while (tasks.size() && std::get<0>(tasks.front()) <= now) {
      std::pop_heap(tasks.begin(), tasks.end(),
                    [](const Task &a, const Task &b) { return std::get<0>(a) < std::get<0>(b); });
      std::get<2>(tasks.back())(std::get<1>(tasks.back()));
      tasks.pop_back();
    }
  }

  bool reschedule(int taskId, uint64_t timestamp) {
    auto itr =
        std::find_if(tasks.begin(), tasks.end(), [taskId](const Task &task) { return std::get<1>(task) == taskId; });
    if (itr != tasks.end()) {
      std::get<0>(*itr) = timestamp;

      std::make_heap(tasks.begin(), tasks.end(),
                     [](const Task &a, const Task &b) { return std::get<0>(a) < std::get<0>(b); });

      return true;
    }
    return false;
  }

  void cancel(int taskId) {
    tasks.erase(
        std::remove_if(tasks.begin(), tasks.end(), [taskId](const Task &t) { return std::get<1>(t) == taskId; }),
        tasks.end());
    std::make_heap(tasks.begin(), tasks.end(),
                   [](const Task &a, const Task &b) { return std::get<0>(a) < std::get<0>(b); });
  }
};

}  // namespace esphome::wii_balance_board::detail
