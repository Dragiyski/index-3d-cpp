#ifndef COMPILE_MESH_PROGRESS_HXX
#define COMPILE_MESH_PROGRESS_HXX

#include <atomic>
#include <chrono>
#include <cstdint>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

class Progress
{
private:
    std::thread _thread;
    std::condition_variable _print_wait, _task_wait;
    std::mutex _mutex;
    std::string _title;
    std::size_t _total, _done;
    bool _should_update, _is_printing, _has_task;
    std::atomic_bool _is_exiting, _is_running;
    std::chrono::steady_clock::time_point _last_update;
    std::chrono::steady_clock::duration _update_interval;
    static bool _has_atexit;
    static Progress _singleton;

    Progress();
    ~Progress();
public:
    void _main();
    virtual void print(const std::string &title, std::size_t done, std::size_t total) const;
    static Progress& get();
    void update(const std::string &title, std::size_t total = 0);
    void pause();
    void resume();
    bool is_paused() const;
    void task_notify(std::size_t count = 1);
    void done();
};

#endif /* COMPILE_MESH_PROGRESS_HXX */