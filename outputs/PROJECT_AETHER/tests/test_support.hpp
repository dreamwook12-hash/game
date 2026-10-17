#pragma once
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

struct TestSuite {
    int passed=0, failed=0;
    void run(const std::string& name, const std::function<void()>& test) {
        try { test(); ++passed; std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& e) { ++failed; std::cout << "FAIL " << name << ": " << e.what() << '\n'; }
    }
};
inline void check(bool value, const std::string& message) { if (!value) throw std::runtime_error(message); }
inline void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected)>tolerance)
        throw std::runtime_error(message+" expected="+std::to_string(expected)+" observed="+std::to_string(actual));
}
template<class F> inline void rejects(F&& fn, const std::string& message) {
    bool thrown=false; try { fn(); } catch(const std::exception&) { thrown=true; } check(thrown,message);
}
