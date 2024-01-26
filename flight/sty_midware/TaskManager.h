/**
 * @file: TaskManager.h
 * @brief: Task Manager Class decl
 * @author: zhangxin@santypilot
 * @date: 2024-1-26
 */

#ifndef _STY_TASK_MANAGER_H
#define _STY_TASK_MANAGER_H

namespace sty_midware {
class TaskManager {
public:
    static TaskManager* instance() {
        if (g_instance == nullptr) {
        }
    }
};
}

#endif // _STY_TASK_MANAGER_H
