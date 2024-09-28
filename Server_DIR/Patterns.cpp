#include <iostream>
#include <vector>
#include <queue>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>

#ifndef Pattern_CPP
#define Pattern_CPP

/**
 * Active Object for Pipeline Pattern
 * class consists of a task queue, a worker thread, and synchronization primitives to manage task execution.
 * contains its own thread of control and a queue of pending tasks.
 * provides a solid foundation for asynchronously processing tasks in a way
 * that ensures thread safety and efficient resource handling.
 * */
class ActiveObject
{
private:
    std::queue<std::function<void()>> tasks;
    std::mutex mutex;
    std::condition_variable condition; // Notifies that the shared resource is free to access, Used for make a thread sleep until some condition becomes true.
    std::atomic<bool> running{true}; // Ensuring thread safety.
    std::thread worker;              // Represents a single thread of execution.

    // This is the function run by the worker thread.
    // It waits for tasks to be added to the queue and processes them one by one.
    void workerFunction()
    {
        while (running)
        {
            std::function<void()> task;
            {
                // Ensuring that only one thread can access shared resources at a time
                std::unique_lock<std::mutex> lock(mutex); 
                // Instructs the thread to wait until either a task is available in the tasks queue 
                // or the running variable becomes false.
                condition.wait(lock, [this]
                               { return !tasks.empty() || !running; });
                if (!running && tasks.empty())
                    return;
                task = std::move(tasks.front()); // Fetch the task
                tasks.pop();
            }
            task(); // This line executes the task that was previously fetched from the queue.
        }
    }

public:
    ActiveObject() : worker(&ActiveObject::workerFunction, this) {}
    // stops the worker thread and waits for it to join,
    // ensuring that all tasks are completed before the object is destroyed.
    ~ActiveObject()
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            running = false;
        }
        condition.notify_one();
        worker.join();
    }
    /**
     * Adds a new task to the queue. Notifies the worker thread that a new task is available.
     */
    template <class F>
    void enqueue(F &&f)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            tasks.emplace(std::forward<F>(f));
        }
        condition.notify_one();
    }
};

/**
 * Leader-Follower Thread Pool
 * An implementation of the Leader-Follower pattern using a thread pool.
 * The Leader-Follower pattern decouples task submission from task execution using a fixed number of worker threads in a pool.
 * Threads take turns to become the "leader", which picks tasks, while others wait, improving CPU utilization and load balancing.
 */
class LeaderFollowerPool
{
private:
    std::vector<std::thread> threads; // Stores the worker threads.
    std::queue<std::function<void()>> tasks;
    std::mutex mutex; // Mutex for protecting access to the task queue
    std::condition_variable condition; // Notifies that the shared resource is free to access, Used for make a thread sleep until some condition becomes true.
    bool stop;

public:
    // initializes and starts the worker threads
    LeaderFollowerPool(size_t numThreads) : stop(false)
    {// This for loop will iterate numThreads times. Each iteration creates a new thread in the thread pool.
        for (size_t i = 0; i < numThreads; ++i)
        {//Adds a new thread to a vector which stores all the worker threads in the pool.
            threads.emplace_back([this] 
                                 {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->mutex); //Ensures that access to shared data is thread-safe.
                        /**
                         * This line causes the thread to wait until either:
                         * stop is true (indicating that the pool should shut down).
                         * The task queue (tasks) is not empty (indicating that there is a task to process).
                        */
                        this->condition.wait(lock, [this] { return this->stop || !this->tasks.empty(); });
                        if (this->stop && this->tasks.empty()) return;
                        task = std::move(this->tasks.front());
                        this->tasks.pop(); //The task is removed from the queue.
                    }
                    task();
                } });
        }
    }
    // stops the threads and waits for them to join, ensuring clean shutdown
    ~LeaderFollowerPool()
    {
        {
            std::unique_lock<std::mutex> lock(mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread &worker : threads)
        {
            worker.join();
        }
    }
    // Adds a new task to the queue.
    template <class F>
    void enqueue(F &&f)
    {
        {
            std::unique_lock<std::mutex> lock(mutex);
            tasks.emplace(std::forward<F>(f));
        }
        condition.notify_one();
    }
};

#endif