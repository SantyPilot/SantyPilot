/*
 * @brief: a simple argument parser implementation
 * @author: santypilot 
 * @date: 2024-3-26
 */
#ifndef _STY_ARG_PARSE_H
#define _STY_ARG_PARSE_H

#include <iostream>
#include <string>
#include <vector>
#include <map>

namespace components {
class Argument {
public:    
    Argument(const std::string& name = "--INVALID_ARG") :
        _required(false), _names({ name }) {}
    Argument& set_required(bool req) {
        _required = req;
        return *this;
    }
    bool get_required() const {
        return _required;
    }
    Argument& default_values(const std::vector<std::string>& dv) {
        _default_values = dv;
        return *this;
    }
    const std::vector<std::string>& get_default_values() {
        return _default_values;
    }
    void set_values(const std::vector<std::string>& vals) {
        _values = vals;
    }
    template<typename T> 
    T get_values() { // uint64_t & bool
        auto raw = prepare_values("0");
        return std::stoul(raw); 
    }
	/*
    template<> std::string get_values<std::string>();
    template<> double get_values<double>();
    template<> int32_t get_values<int32_t>();
    template<> std::vector<size_t> get_values<std::vector<size_t>>();
    template<> std::vector<double> get_values<std::vector<double>>();
    template<> std::vector<int32_t> get_values<std::vector<int32_t>>();
    template<> std::vector<std::string> get_values<std::vector<std::string>>();
	*/

    Argument& set_desc(const std::string& desc) {
        _desc = desc;
        return *this;
    }
    const std::string& get_desc() {
        return _desc;
    }
    const std::string real_name();
    void add_name(const std::string& name);
    const std::string longest_name();
    const std::vector<std::string>& get_names();

    // TODO: check valid && print correct usage

private:
    std::string prepare_values(const std::string& weak_val) {
        std::string raw = weak_val;
        if (_values.empty()) {
            if (!_default_values.empty()) {
                raw = _default_values.front();
            } else {
                raw = weak_val.front();
            }
        } else {
            raw = _values.front();
        }
        return raw;
    }
    std::vector<std::string> 
        prepare_values(const std::vector<std::string>& weak_val) {
        auto raw = weak_val;
        if (_values.empty()) {
            if (!_default_values.empty()) {
                raw = _default_values;
            } else {
                raw = _default_values;
            }
        } else {
            raw = _values;
        }
        return raw;
    }
    bool _required;
    std::vector<std::string> _default_values;
    std::vector<std::string> _values;
    std::vector<std::string> _names;
    std::string _desc;
};

//--- Argument
template<>
std::string Argument::get_values<std::string>() { // std::string
    auto raw = prepare_values("0");
    return raw;
}

template<>
double Argument::get_values<double>() { // double
    auto raw = prepare_values("0.0");
    return std::stod(raw);
}
template<>
int32_t Argument::get_values<int32_t>() { // int32_t
    auto raw = prepare_values("0");
    return std::stoi(raw);
}
template<> // std::vector<uint64_t>
std::vector<size_t> Argument::get_values<std::vector<size_t>>() {
    std::vector<std::string> weak = { "0" };
    auto raw = prepare_values(weak);
    std::vector<size_t> ret;
    ret.resize(raw.size());
    for (size_t i = 0; i < raw.size(); i++) {
        ret[i] = stoul(raw[i]);
    }
    return ret;
}
template<> // std::vector<double>
std::vector<double> Argument::get_values<std::vector<double>>() {
    std::vector<std::string> weak = { "0" };
    auto raw = prepare_values(weak);
    std::vector<double> ret;
    ret.resize(raw.size());
    for (size_t i = 0; i < raw.size(); i++) {
        ret[i] = stod(raw[i]);
    }
    return ret;
}
template<> // std::vector<int32_t>
std::vector<int32_t> Argument::get_values<std::vector<int32_t>>() {
    std::vector<std::string> weak = { "0" };
    auto raw = prepare_values(weak);
    std::vector<int32_t> ret;
    ret.resize(raw.size());
    for (size_t i = 0; i < raw.size(); i++) {
        ret[i] = stoi(raw[i]); // TODO: opt this
    }
    return ret;
}
template<> // std::vector<std::string>
std::vector<std::string> Argument::get_values<std::vector<std::string>>() {
    std::vector<std::string> weak = { "0" };
    auto raw = prepare_values(weak);
    return raw;
}


class ArgumentParser {
public:
	// not support implicit convert
	explicit ArgumentParser(std::string name = "SANTYPILOT_ARG_PARSER") : _name(name) {
        _argument_list.resize(MAX_ARGS_NUM + 1/* --help */);
        add_argument("-h", "--help");
    }
	~ArgumentParser() = default;
	// ban copy/move semantic
	ArgumentParser(const ArgumentParser& other) = delete;
	ArgumentParser& operator=(const ArgumentParser& other) = delete;
	ArgumentParser(ArgumentParser&&) noexcept = delete;
	ArgumentParser& operator=(ArgumentParser&&) = delete;

	void parse_args(int argc, const char* const argv[]) {
		parse_args({ argv, argv + argc });
	}
    void parse_args(const std::vector<std::string>& arguments);
	void parse_xml(const std::string& xml);
    // Note: only support string input!
    template<typename... T>
    Argument& add_argument(const std::string& name, const T&... names) {
        _argument_map[name] = &add_argument(names ...);
        return _argument_list.back();
    }
    Argument& add_argument(const std::string& name) {
        _argument_list.emplace_back(name);
        _argument_map[name] = &_argument_list.back();
        return _argument_list.back();
    }
    bool has(const std::string& name);
    template<typename T>
    T get(const std::string& name) {
        if (_argument_map.count(name) <= 0) {
            return T();
        }
        return _argument_map[name]->get_values<T>();
    }
    void help();
private:
    /* Note: args cannot start with internal key! */
    const size_t MAX_ARGS_NUM = 20;
    const std::string PREFIX_CHAR{ "-" };
    const std::string ASSIGN_CHAR{ "=" };
    std::string _name; // app name
    std::map<std::string, Argument*> _argument_map;
    std::vector<Argument> _argument_list;
    size_t _cur_idx = 0;
};
} // components 

#endif // _STY_ARG_PARSE_H
