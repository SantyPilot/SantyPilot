/**
 * @file: DemoTask.h
 * @brief: Simple Demo Task 
 * @author: zhangxin@santypilot
 * @date: 2024-1-26
 */
#ifndef _DEMO_TASK_H
#define _DEMO_TASK_H
#include "sty_midware/TaskBase.h"
#include <string>

const std::string DEMO_TASK_NAME = "DEMO_TASK";
REGISTER_STY_MODULE(DEMO_TASK_NAME, DemoTask);

namespace sty_modules {
class DemoTask: public TaskBase {
public:
    DemoTask(): TaskBase(DEMO_TASK_NAME) {}
    virtual bool init() {}
    virtual void run() {
        while (!should_exit) {
            // TODO: wrap delay interface
        }
    }
private:
    size_t priority;
};
}

#endif // _DEMO_TASK_H
