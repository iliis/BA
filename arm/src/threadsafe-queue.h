#ifndef THREADSAFE_QUEUE_H_
#define THREADSAFE_QUEUE_H_

#include <pthread.h>
#include <queue>
#include <string>

namespace threadsafe {

template<typename QueueType>
class ThreadSafeQueue {
  friend bool test_funcs(void* (*)(void*), void* (*)(void*), const std::string&,
                         bool);

 public:
  void NotifyAll() {
    pthread_cond_broadcast(&condition_empty_);
    pthread_cond_broadcast(&condition_full_);
  }

  ThreadSafeQueue() {
    shutdown_ = false;
    pthread_mutex_init(&mutex_, NULL);
    pthread_cond_init(&condition_empty_, NULL);
    pthread_cond_init(&condition_full_, NULL);
  }

  ~ThreadSafeQueue() {
    shutdown_ = true;
    NotifyAll();
    pthread_mutex_destroy(&mutex_);
    pthread_cond_destroy(&condition_empty_);
    pthread_cond_destroy(&condition_full_);
  }

  void Shutdown() {
    shutdown_ = true;
    NotifyAll();
  }

  // Push to the queue.
  void Push(const QueueType& value) {
    PushNonBlocking(value);
  }

  // Push to the queue.
  void PushNonBlocking(const QueueType& value) {
    pthread_mutex_lock(&mutex_);
    queue_.push(value);
    pthread_cond_signal(&condition_empty_);  // Signal that data is available.
    pthread_mutex_unlock(&mutex_);
  }

  size_t Size() {
    pthread_mutex_lock(&mutex_);
    size_t size = queue_.size();
    pthread_mutex_unlock(&mutex_);
    return size;
  }

  // Push to the queue if the size is less than max_queue_size, else block.
  void PushBlockingIfFull(const QueueType& value, size_t max_queue_size) {
    while (true) {
      pthread_mutex_lock(&mutex_);
      size_t size = queue_.size();
      if (size >= max_queue_size) {
        pthread_cond_wait(&condition_full_, &mutex_);
      }
      if (size >= max_queue_size) {
        pthread_mutex_unlock(&mutex_);
        continue;
      }
      queue_.push(value);
      pthread_cond_signal(&condition_empty_);  // Signal that data is available.
      pthread_mutex_unlock(&mutex_);
      break;
    }
  }

  // Returns true if oldest was dropped because queue was full.
  bool PushNonBlockingDroppingIfFull(const QueueType& value,
                                     size_t max_queue_size) {
    pthread_mutex_lock(&mutex_);
    bool result = false;
    if (queue_.size() >= max_queue_size) {
      queue_.pop();
      result = true;
    }
    queue_.push(value);
    pthread_cond_signal(&condition_empty_);  // Signal that data is available.
    pthread_mutex_unlock(&mutex_);
    return result;
  }

  // Pops from the queue blocking if queue is empty.
  bool Pop(QueueType* value) {
    return PopBlocking(value);
  }

  // Pops from the queue blocking if queue is empty.
  bool PopBlocking(QueueType* value) {
    assert(value);
    while (!shutdown_) {
      pthread_mutex_lock(&mutex_);
      if (queue_.empty()) {
        pthread_cond_wait(&condition_empty_, &mutex_);
      }
      if (queue_.empty()) {
        pthread_mutex_unlock(&mutex_);
        continue;
      }
      QueueType _value = queue_.front();
      queue_.pop();
      pthread_cond_signal(&condition_full_);  // Notify that space is available.
      pthread_mutex_unlock(&mutex_);
      *value = _value;
      return true;
    }
    return false;
  }

  // Check queue is empty, if yes return false, not altering value. If queue not
  // empty update value and return true.
  bool PopNonBlocking(QueueType* value) {
    assert(value);
    pthread_mutex_lock(&mutex_);
    if (queue_.empty()) {
      pthread_mutex_unlock(&mutex_);
      return false;
    }
    *value = queue_.front();
    queue_.pop();
    pthread_mutex_unlock(&mutex_);
    return true;
  }
  pthread_mutex_t mutex_;
  pthread_cond_t condition_empty_;
  pthread_cond_t condition_full_;
  std::queue<QueueType> queue_;
  bool shutdown_;
};

}  // namespace threadsafe

#endif  // THREADSAFE_QUEUE_H_
