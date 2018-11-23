#pragma once
#include <vector>
#include <tuple>
#include <alogrithm>

#include "function.hpp"

// -------------------
// tunable parameters
// -------------------
#define VAR_RELATIVE_SPEED_BOUND 15 // m/s
#define VAR_ACC_BOUND 9 // m/s^2
#define NUM_SAMPLE 100

class Fuzzy 
{
  public:
    Fuzzy()
    {
        // init rule base
        // TODO
        rule_base.emplace_back(std::make_tupple(ContinuousFunction()))

    }

    float operator()(float relative_speed, float distance_ratio)
    {
        // inference each rule in the rule base
        SampledFunction<NUM_SAMPLE> result; 
        for(auto rule : rule_base)
        {
            // get bound
            float bound = std::min(
                std::get<0>(rule).get_value(relative_speed)
              , std::get<1>(rule).get_value(distance_ratio));

            // bound the output fuzzy set and min with result
            result.fuzzy_max(std::get<2>(rule).bound(bound));
        }

        // defuzzification and output my acceleration
        return result.defuzzification()
    }

  private:
    std::vector<
        std::tuple<
            ContinuousFunction
          , ContinuousFunction
          , SampledFunction<NUM_SAMPLE>
          >> rule_base;
};
