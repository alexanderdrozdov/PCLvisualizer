#include <queue>
#include <mutex>
#include <condition_variable>

template<typename T>
class SafeQueue {
public:
    void push(const T& value) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(value);
        m_cond.notify_one();
    }

    bool pop(T& value, bool wait = true) {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (wait) {
            m_cond.wait(lock, [this] { return !m_queue.empty(); });
        } else {
            if (m_queue.empty()) return false;
        }
        value = m_queue.front();
        m_queue.pop();
        return true;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

private:
    std::queue<T> m_queue;
    mutable std::mutex m_mutex;
    std::condition_variable m_cond;
};