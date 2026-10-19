#pragma once

#include <chrono>
#include <pthread.h>
#include <stdexcept>
#include <system_error>

// A low-priority RPC must not indefinitely block the realtime control thread.
class PriorityMutex {
 public:
  PriorityMutex() {
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    const int protocol = pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    const int initialized = protocol ? protocol : pthread_mutex_init(&mutex_, &attr);
    pthread_mutexattr_destroy(&attr);
    if (initialized) throw std::system_error(initialized, std::generic_category());
  }
  ~PriorityMutex() { pthread_mutex_destroy(&mutex_); }
  PriorityMutex(const PriorityMutex&) = delete;
  PriorityMutex& operator=(const PriorityMutex&) = delete;
  void lock() {
    const int error = pthread_mutex_lock(&mutex_);
    if (error) throw std::system_error(error, std::generic_category());
  }
  bool try_lock() { return pthread_mutex_trylock(&mutex_) == 0; }
  template<class Rep, class Period>
  bool try_lock_for(std::chrono::duration<Rep, Period> duration) {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        (std::chrono::system_clock::now() + duration).time_since_epoch()).count();
    const timespec deadline{ns / 1000000000, ns % 1000000000};
    return pthread_mutex_timedlock(&mutex_, &deadline) == 0;
  }
  void unlock() { pthread_mutex_unlock(&mutex_); }
 private:
  pthread_mutex_t mutex_;
};
