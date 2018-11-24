#pragma once
#include <vector>
#include <tuple>
#include <algorithm>

#include "function.hpp"

// -------------------
// tunable parameters
// -------------------
#define VAR_RELATIVE_SPEED_BOUND 16 // m/s 
#define VAR_ACC_BOUND 8 // m/s^2 
#define NUM_SAMPLE 10


#define NUM_SLICE 4
#define NUM_SPEED_SET 2*NUM_SLICE-1
#define NUM_DISTANCE_SET 2*NUM_SLICE-1
#define NUM_ACC_SET 2*NUM_SLICE-1

class Fuzzy 
{
  public:
    Fuzzy()
    {
        // init distance fuzzy set
        fuzzy_set_distance_.emplace_back(
                ContinuousFunction(0.5, 0.25)); //NL
        fuzzy_set_distance_.emplace_back(
                ContinuousFunction(0.25, 0.5, 0.75)); //NM
        fuzzy_set_distance_.emplace_back(
                ContinuousFunction(0.5, 0.75, 1.0)); //NS
        fuzzy_set_distance_.emplace_back(
                ContinuousFunction(0.75, 1.0, 1.25)); //AZ
        fuzzy_set_distance_.emplace_back(
                ContinuousFunction(1.0, 1.25, 1.5)); //PS
        fuzzy_set_distance_.emplace_back(
                ContinuousFunction(1.25, 1.5, 1.75)); //PM
        fuzzy_set_distance_.emplace_back(
                ContinuousFunction(1.5, 1.75)); //PL

        // init speed fuzzy set
        fuzzy_set_speed_.emplace_back(
            ContinuousFunction(
                (-0.5)*VAR_RELATIVE_SPEED_BOUND 
              , (-0.75)*VAR_RELATIVE_SPEED_BOUND)); //NL
        fuzzy_set_speed_.emplace_back(
            ContinuousFunction(
                (-0.75)*VAR_RELATIVE_SPEED_BOUND 
              , (-0.5)*VAR_RELATIVE_SPEED_BOUND 
              , (-0.25)*VAR_RELATIVE_SPEED_BOUND)); //NM
        fuzzy_set_speed_.emplace_back(
            ContinuousFunction(
                (-0.5)*VAR_RELATIVE_SPEED_BOUND 
              , (-0.25)*VAR_RELATIVE_SPEED_BOUND //NS
              , 0.0)); 
        fuzzy_set_speed_.emplace_back(
            ContinuousFunction(
                (-0.25)*VAR_RELATIVE_SPEED_BOUND
              , 0.0
              , (0.25)*VAR_RELATIVE_SPEED_BOUND)); //AZ
        fuzzy_set_speed_.emplace_back(
            ContinuousFunction(
                0.0
              , (0.25)*VAR_RELATIVE_SPEED_BOUND
              , (0.5)*VAR_RELATIVE_SPEED_BOUND)); //PS
        fuzzy_set_speed_.emplace_back(
            ContinuousFunction(
                (0.25)*VAR_RELATIVE_SPEED_BOUND
              , (0.5)*VAR_RELATIVE_SPEED_BOUND
              , (0.75)*VAR_RELATIVE_SPEED_BOUND)); //PM
        fuzzy_set_speed_.emplace_back(
            ContinuousFunction(
                (0.5)*VAR_RELATIVE_SPEED_BOUND
              , (0.75)*VAR_RELATIVE_SPEED_BOUND)); //PL

        // init output acceleration fuzzy set
        fuzzy_set_acc_.emplace_back( //NL
            SampledFunction<NUM_SAMPLE, VAR_ACC_BOUND>(
                (-0.5)*VAR_ACC_BOUND 
              , (-0.75)*VAR_ACC_BOUND));
        fuzzy_set_acc_.emplace_back( //NM
            SampledFunction<NUM_SAMPLE, VAR_ACC_BOUND>(
                (-0.75)*VAR_ACC_BOUND
              , (-0.5)*VAR_ACC_BOUND
              , (-0.25)*VAR_ACC_BOUND));
        fuzzy_set_acc_.emplace_back( //NS
            SampledFunction<NUM_SAMPLE, VAR_ACC_BOUND>(
                (-0.5)*VAR_ACC_BOUND
              , (-0.25)*VAR_ACC_BOUND
              , 0));
        fuzzy_set_acc_.emplace_back( //AZ
            SampledFunction<NUM_SAMPLE, VAR_ACC_BOUND>(
                (-0.25)*VAR_ACC_BOUND
              , 0
              , (0.25)*VAR_ACC_BOUND));
        fuzzy_set_acc_.emplace_back( //PS
            SampledFunction<NUM_SAMPLE, VAR_ACC_BOUND>(
                0
              , (0.25)*VAR_ACC_BOUND
              , (0.5)*VAR_ACC_BOUND));
        fuzzy_set_acc_.emplace_back( //PM
            SampledFunction<NUM_SAMPLE, VAR_ACC_BOUND>(
                (0.25)*VAR_ACC_BOUND
              , (0.5)*VAR_ACC_BOUND
              , (0.75)*VAR_ACC_BOUND));
        fuzzy_set_acc_.emplace_back( //PL
            SampledFunction<NUM_SAMPLE, VAR_ACC_BOUND>(
                (0.5)*VAR_ACC_BOUND
              , (0.75)*VAR_ACC_BOUND));

        // init rule base [distance][speed]
        rule_grid_ = { {0, 0, 0, 0, 1, 2, 2}
                     , {0, 0, 0, 0, 1, 3, 3}
                     , {1, 1, 1, 2, 3, 5, 5}
                     , {1, 1, 2, 3, 4, 5, 5}
                     , {1, 1, 3, 4, 5, 5, 5}
                     , {3, 3, 5, 6, 6, 6, 6}
                     , {4, 4, 5, 6, 6, 6, 6}
                     };
    }

    float operator()(float relative_speed, float distance_ratio)
    {
        // inference each rule in the rule base
        SampledFunction<NUM_SAMPLE, VAR_ACC_BOUND> result;
        for(auto i=0; i<NUM_SPEED_SET; i++)
            for(auto j=0; j<NUM_DISTANCE_SET; j++)
            {
                // get bound
                float bound = std::min(
                   fuzzy_set_distance_[i].get_value(distance_ratio)
                 , fuzzy_set_speed_[j].get_value(relative_speed));

                // bound the output fuzzy set and min with result
                result.fuzzy_max(
                    fuzzy_set_acc_[rule_grid_[i][j]].bound(bound));
            }

        // defuzzification and output my acceleration
        return result.defuzzification();
    }

  private:
    std::vector<ContinuousFunction> fuzzy_set_speed_;

    std::vector<ContinuousFunction> fuzzy_set_distance_;

    std::vector<
        SampledFunction<NUM_SAMPLE, VAR_ACC_BOUND>> fuzzy_set_acc_;

    std::vector<std::vector<int>> rule_grid_;
};
