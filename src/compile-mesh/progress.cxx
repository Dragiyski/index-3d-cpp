#include "progress.hxx"
#include <iostream>
using namespace std::chrono_literals;

bool Progress::_has_atexit = false;
Progress Progress::_singleton;

Progress::Progress() : _thread(&Progress::_main, this), _print_wait(), _task_wait(), _mutex(), _title(""), _total{0}, _done{0}, _should_update{false}, _is_printing{false}, _is_exiting(false), _is_running(false), _has_task(false), _last_update(), _update_interval(1000ms)
{
}

Progress::~Progress()
{
    _is_exiting.store(true, std::memory_order::release);
    _task_wait.notify_all();
    if (_thread.joinable()) {
        _thread.join();
    }
}

void Progress::_main()
{
    std::unique_lock lock(_mutex);
    while (true) {
        if (_is_exiting.load(std::memory_order::consume)) {
            return;
        }
        if (_has_task && _is_running.load(std::memory_order::consume)) {
            auto now = std::chrono::steady_clock::now();
            if (_should_update || now - _last_update >= _update_interval) {
                _should_update = false;
                _last_update = now;
                print(_title, _done, _total);
                if (_total > 0 && _done == _total) {
                    _has_task = false;
                }
            }
        }
        if (_is_printing) {
            _is_printing = false;
            _print_wait.notify_all();
        }
        _task_wait.wait_for(lock, _update_interval);
    }
}

void Progress::print(const std::string &title, std::size_t done, std::size_t total) const
{
    std::cerr << title << " " << "(" << done;
    if (total > 0) {
        std::cerr << " / " << total;
    }
    std::cerr << ")" << std::endl;
}

Progress& Progress::get()
{
    return _singleton;
}

void Progress::update(const std::string &title, std::size_t total)
{
    std::unique_lock lock(_mutex);
    if (_has_task && _is_running.load(std::memory_order::consume)) {
        _should_update = true;
        _task_wait.notify_all();
        _print_wait.wait(lock, [this]()->bool{ return !_is_printing; });
    }
    _should_update = true;
    _title = title;
    _total = total;
    _done = 0;
    _is_running.store(true, std::memory_order::release);
    _has_task = true;
    _task_wait.notify_all();
    _is_printing = true;
    _print_wait.wait(lock, [this]()->bool{ return !_is_printing; });
}

void Progress::pause()
{
    _is_running.store(false, std::memory_order::release);
}

void Progress::resume()
{
    _is_running.store(true, std::memory_order::release);
}

bool Progress::is_paused() const
{
    return _is_running.load(std::memory_order::consume);
}

void Progress::task_notify(std::size_t count)
{
    _done += count;
}

void Progress::done()
{
    std::unique_lock lock(_mutex);
    if (_has_task && _is_running.load(std::memory_order::consume)) {
        _should_update = true;
        _is_printing = true;
        _task_wait.notify_all();
        _print_wait.wait(lock, [this]()->bool{ return !_is_printing; });
    }
    _has_task = false;
}
