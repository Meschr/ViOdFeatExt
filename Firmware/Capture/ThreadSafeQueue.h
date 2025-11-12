#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

// A thread-safe wrapper around std::queue
template <typename T> class ThreadSafeQueue {
public:
  void push(T value) {
    // Lock the queue to add an item
    std::lock_guard<std::mutex> lock(m_mutex);
    m_queue.push(std::move(value));
    // Notify one waiting thread that there is data
    m_cond.notify_one();
  }

  // Waits until an item is available and pops it
  T wait_and_pop() {
    // Acquire the lock
    std::unique_lock<std::mutex> lock(m_mutex);
    // Wait until the queue is not empty. The lambda prevents spurious wakeups.
    m_cond.wait(lock, [this] { return !m_queue.empty(); });
    T value = std::move(m_queue.front());
    m_queue.pop();
    return value;
  }

  bool empty() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_queue.empty();
  }

private:
  std::queue<T> m_queue;
  std::mutex m_mutex;
  std::condition_variable m_cond;
};
