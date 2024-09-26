/*
 * @brief: a simple argument parser implementation
 *  Note: this implementation is not as good
 *  as it stores string which is memory unfriendly
 *  maybe we should stores as char* compact mem
 * @author: santypilot 
 * @date: 2024-3-27
 */

#include "StyArgParser.h"
#include "StyString.h"
#include "StyLog.h"
#include <set>

namespace components {
const std::string Argument::real_name() {
    auto name = longest_name();
    auto idx = name.find_first_not_of('-');
    if (idx == std::string::npos) { 
        return std::string();
    }
    return _names[0].substr(idx);
}

void Argument::add_name(const std::string& name) {
    _names.emplace_back(name);
    return;
}

const std::string Argument::longest_name() {
    size_t idx = 0;
    size_t len = 0;
    for (auto i = 0; i < _names.size(); i++) {
        if (_names[i].size() <= len) {
            continue;
        }
        len = _names[i].size();
        idx = i;
    }
    return _names[idx];
}

const std::vector<std::string>& Argument::get_names() {
    return _names;
}

//--- ArgumentParser
bool ArgumentParser::has(const std::string& name) {
    return (_argument_map.count(name) > 0);
}
void ArgumentParser::help() {
    if (_argument_map.size() == 0) {
        LOG(INFO) << _name << " does not support any input arguments!";
        return;
    }
    LOG(INFO) << _name << " usage;";
    std::set<Argument*> tb;
    std::string hint = "";
    for (auto& kv : _argument_map) {
        if (kv.second == nullptr) {
            continue;
        }
        if (tb.count(kv.second) > 0) {
            continue;
        }
        auto& names = kv.second->get_names();
        for (auto i = 0; i < names.size(); i++) {
            hint += names[i];
            if (i != names.size() - 1) {
                hint += " ";
            }
        }
        auto& desc = kv.second->get_desc();
        hint += " " + desc;
    }
}
void ArgumentParser::parse_args(const std::vector<std::string>& arguments) {
    // 1.split
    using arg_kv_t = std::pair<std::string, std::vector<std::string>>;
    std::vector<arg_kv_t> valid_grps;
    const auto has_prefix =
        [this](const std::string& raw) -> bool {
        if (raw.empty()) {
            return false;
        }
        return PREFIX_CHAR.find(raw[0]) != std::string::npos;
        };
    bool peek = false;
    arg_kv_t grp;
    for (auto idx = 1/* skip 0 exe file */; idx < arguments.size();) {
        // 1. input valid
        auto arg = components::trim_spaces(arguments[idx]);
        // 2. start with - or --
        if (has_prefix(arg)) {
            idx++;
            if (_argument_map.count(arg) <= 0) {
                continue;
            }
            grp = { arg, {} }; // allocate
            peek = true;
        }
        // 3. peek one or more into groups
        while (!has_prefix(arg) &&
            peek && idx < arguments.size()) {
            grp.second.emplace_back(arg);
            idx++;
            if (idx >= arguments.size()) {
                break;
            }
            arg = components::trim_spaces(arguments[idx]);
        }
        if (!grp.second.empty()) {
            valid_grps.emplace_back(grp);
            peek = false;
        }
        // avoid dead cycle
        if (!has_prefix(arg) && !peek) {
            idx++;
        }
    }
    // 2. set groups
    for (auto& grp : valid_grps) {
        auto& name = grp.first;
        _argument_map[name]->set_values(grp.second);
    }
    return;
}
} // components 
