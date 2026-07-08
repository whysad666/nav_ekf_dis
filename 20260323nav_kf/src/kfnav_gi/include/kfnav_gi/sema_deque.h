#pragma once
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <deque>
#include <mutex>

template <typename T> class SemaDeque {
public:
    SemaDeque()
        : sema(0), push_cnt(0), pop_cnt(0) {
    }
    
    void push(T data) {
        std::lock_guard<std::mutex> lock(mut);
        //进入临界区
        queue.push_back(std::move(data));
        sema.post();
        push_cnt++;
    }
    
    auto pop() -> T {
        sema.wait();
        std::lock_guard<std::mutex> lock(mut);
        //进入临界区 - 现在信号量确保至少有一个元素
        T data = std::move(queue.front());
        queue.pop_front();
        pop_cnt++;
        return data;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mut);
        queue.clear();

        // 重置信号量到初始状态（值为0）
        // 限制尝试次数以避免无限循环
        size_t estimated_size = push_cnt - pop_cnt;
        for(size_t i = 0; i < estimated_size; ++i) {
            if(!sema.try_wait()) {
                break;  // 没有更多信号量许可了
            }
        }

        // 重置计数器
        push_cnt = 0;
        pop_cnt = 0;
    }

    auto at(size_t index) -> T {
        std::lock_guard<std::mutex> lock(mut);
        return queue.at(index);
    }

    auto size() -> size_t {
        std::lock_guard<std::mutex> lock(mut);
        return queue.size();
    }

    auto empty() -> bool {
        return size() == 0;
    }

    auto front() -> T {
        std::lock_guard<std::mutex> lock(mut);
        return queue.front();
    }

    auto back() -> T {
        std::lock_guard<std::mutex> lock(mut);
        return queue.back();
    }

    auto getPushCnt() -> size_t {
        std::lock_guard<std::mutex> lock(mut);
        return push_cnt;
    }
    
    auto getPopCnt() -> size_t {
        std::lock_guard<std::mutex> lock(mut);
        return pop_cnt;
    }

    // 删除拷贝和移动构造函数，因为包含不可拷贝的同步原语
    SemaDeque(const SemaDeque&) = delete;
    SemaDeque& operator=(const SemaDeque&) = delete;
    SemaDeque(SemaDeque&&) = delete;
    SemaDeque& operator=(SemaDeque&&) = delete;

private:
    size_t push_cnt{0};
    size_t pop_cnt{0};
    std::mutex mut;
    boost::interprocess::interprocess_semaphore sema;
    std::deque<T> queue; // 数据队列
};
