#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

namespace vectorwatch {

// Keeps a fixed group of worker threads alive and gives them prediction jobs.
// The destructor stops and joins every thread automatically.
class PredictionWorkerPool {
public:
    explicit PredictionWorkerPool(std::size_t workerCount);
    ~PredictionWorkerPool();

    PredictionWorkerPool(const PredictionWorkerPool&) = delete;
    PredictionWorkerPool& operator=(const PredictionWorkerPool&) = delete;
    PredictionWorkerPool(PredictionWorkerPool&&) = delete;
    PredictionWorkerPool& operator=(PredictionWorkerPool&&) = delete;

    std::size_t size() const noexcept;
    std::future<void> submit(std::function<void()> work);

private:
    mutable std::mutex mutex_{};
    std::condition_variable workAvailable_{};
    std::deque<std::packaged_task<void()>> workQueue_{};
    bool stopping_{};
    std::vector<std::thread> workers_{};

    void run();
    void stopAndJoin() noexcept;
};

} // namespace vectorwatch
