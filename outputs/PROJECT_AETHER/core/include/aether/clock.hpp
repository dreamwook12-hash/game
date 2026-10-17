#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace aether {
struct ClockAdvance {
    std::uint32_t steps=0;
    double step_seconds=0,interpolation_alpha=0,dropped_seconds=0;
};
class FixedStepClock {
public:
    explicit FixedStepClock(double step_seconds=1.0/120.0,std::uint32_t max_catchup_steps=8)
        : step_seconds_(step_seconds),max_catchup_steps_(max_catchup_steps) {
        if (!std::isfinite(step_seconds) || step_seconds<1e-6 || step_seconds>1.0)
            throw std::invalid_argument("Fixed step must be in [1e-6, 1] seconds");
        if (max_catchup_steps==0 || max_catchup_steps>1000000)
            throw std::invalid_argument("Catch-up limit must be in [1, 1000000]");
    }
    ClockAdvance advance(double elapsed_seconds) {
        if (!std::isfinite(elapsed_seconds) || elapsed_seconds<0 || elapsed_seconds>86400)
            throw std::invalid_argument("Frame elapsed time must be in [0, 86400] seconds");
        const long double accumulated=accumulator_+static_cast<long double>(elapsed_seconds);
        const long double count=std::floor(accumulated/step_seconds_+1e-12L);
        const auto whole_steps=static_cast<std::uint64_t>(count);
        const auto executed=static_cast<std::uint32_t>(std::min<std::uint64_t>(whole_steps,max_catchup_steps_));
        if (total_steps_>std::numeric_limits<std::uint64_t>::max()-executed)
            throw std::overflow_error("Simulation tick counter overflow");
        accumulator_=std::max(0.0L,accumulated-count*step_seconds_);
        const double dropped=static_cast<double>((whole_steps-executed)*static_cast<long double>(step_seconds_));
        total_steps_+=executed;
        dropped_seconds_+=dropped;
        const double alpha=std::clamp(static_cast<double>(accumulator_/step_seconds_),0.0,
                                      std::nextafter(1.0,0.0));
        return {executed,step_seconds_,alpha,dropped};
    }
    void reset() { accumulator_=0; total_steps_=0; dropped_seconds_=0; }
    double stepSeconds() const { return step_seconds_; }
    std::uint64_t totalSteps() const { return total_steps_; }
    double totalDroppedSeconds() const { return dropped_seconds_; }
private:
    double step_seconds_;
    std::uint32_t max_catchup_steps_;
    long double accumulator_=0;
    std::uint64_t total_steps_=0;
    double dropped_seconds_=0;
};
} // namespace aether
