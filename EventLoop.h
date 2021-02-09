//
// Created by Andrey Pahomov on 09.02.21.
//

#ifndef EYE_EVENTLOOP_H
#define EYE_EVENTLOOP_H

#include <atomic>
#include <functional>
#include <list>
#include <mutex>

struct event_base;

class EventLoop
{
public:
    EventLoop();
    ~EventLoop();

    void exit(int code = 0);
    int exec();

    void timeout(unsigned int sec);
    void callLater(const std::function<void()> &func);

private:
    static EventLoop *instance_;

    static void timeoutTriggered(int fd, short event, void *arg);

    struct event_base *event_;
    std::atomic<bool> exit_;
    int exitCode_;

    std::list<std::function<void()>> calls_;
    std::mutex lock_;

    void interrupt();
    void dispatchCalls();
};



#endif //EYE_EVENTLOOP_H
