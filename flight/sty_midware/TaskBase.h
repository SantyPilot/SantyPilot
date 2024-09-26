/**
 * @file: TaskBase.h
 * @brief: Task Base Class decl
 * @author: zhangxin@santypilot
 * @date: 2024-1-26
 */

#ifndef _STY_TASKBASE_H
#define _STY_TASKBASE_H

namespace sty_midware {
class TaskBase {
public:
    TaskBase(): _name("") {}
    TaskBase(const std::string& name): _name(name) {}
    virtual ~TaskBase() {}
    /* do uavobject init */
    virtual bool init() = 0;
    virtual void run() = 0;
private:
    TaskBase(const TaskBase&) = delete;
    TaskBase& operator=() = delete;
    std::string _name;
};
} // sty_midware
#endif //_STY_TASKBASE_H
