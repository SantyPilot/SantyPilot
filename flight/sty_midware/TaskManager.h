/**
 * @file: TaskManager.h
 * @brief: Task Manager Class decl
 * @author: zhangxin@santypilot
 * @date: 2024-1-26
 */

#ifndef _STY_TASK_MANAGER_H
#define _STY_TASK_MANAGER_H
#include <string>
#include "TaskBase.h"

namespace sty_midware {

class TaskManager {
public:
    static TaskManager* instance() {
        if (g_instance == nullptr) {
            g_instance = new TaskManager;
        }
        return g_instance;
    }
    void registMod(const std::string& name, TaskBase* mod) {
        _modules_list[name] = mod;
    }
    bool fetchMod(const std::string& name) {
        if (_modules_list.count(name) < 0) {
            return false;
        }
        auto* mod = _modules_list[name];
        return mod;
    }
    void initMods() {
        for (auto& mods: _modules_list) {
            // mods->init();
        }
    }
    void statistics() {}
private:
    TaskManager() {}
    ~TaskManager() {}
    TaskManager(const TaskManager&) = delete;
    TaskManager& operator=() = delete;

    static TaskManager* g_instance;
    std::map<std::string, TaskBase*> _modules_list;
};

class TaskRegister {
public:
    TaskRegister(const std::string& name, TaskCreateFunc func) {
        auto* base = func();
        TaskManager::instance()->registMod(name, func);
    }
};
} // sty_midware


#define REGISTER_MODULE(name, ptr)                \
    TaskBase* create_##name_ins() {               \
        return new ptr;                           \
    }                                             \
    TaskRegister g_##name_register(name,          \
            create_##name_ins);

#endif // _STY_TASK_MANAGER_H

