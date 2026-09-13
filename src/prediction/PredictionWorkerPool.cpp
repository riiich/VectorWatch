#include "vectorwatch/prediction/PredictionWorkerPool.hpp"

#include <stdexcept>
#include <utility>

namespace vectorwatch {

PredictionWorkerPool::PredictionWorkerPool(std::size_t workerCount) {
    if (workerCount == 0) {
        throw std::invalid_argument{"Prediction worker count must be positive"};
    }
    workers_.reserve(workerCount);
    try {
        for (std::size_t index = 0; index < workerCount; ++index) {
            workers_.emplace_back([this] { run(); });
        }
    } catch (...) {
        stopAndJoin();
        throw;
    }
}

PredictionWorkerPool::~PredictionWorkerPool() {
    stopAndJoin();
}

void PredictionWorkerPool::stopAndJoin() noexcept {
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        stopping_ = true;
    }
    workAvailable_.notify_all();
    for (std::thread& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

std::size_t PredictionWorkerPool::size() const noexcept {
    return workers_.size();
}

std::future<void> PredictionWorkerPool::submit(std::function<void()> work) {
    std::packaged_task<void()> task{std::move(work)};
    std::future<void> completion = task.get_future();
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        if (stopping_) {
            throw std::runtime_error{"Prediction worker pool is stopping"};
        }
        workQueue_.push_back(std::move(task));
    }
    workAvailable_.notify_one();
    return completion;
}

void PredictionWorkerPool::run() {
    while (true) {
        std::packaged_task<void()> work;
        {
            std::unique_lock<std::mutex> lock{mutex_};
            workAvailable_.wait(
                lock,
                [this] { return stopping_ || !workQueue_.empty(); });
            if (stopping_ && workQueue_.empty()) {
                return;
            }
            if (workQueue_.empty()) {
                continue;
            }
            work = std::move(workQueue_.front());
            workQueue_.pop_front();
        }
        work();
    }
}

} // namespace vectorwatch
