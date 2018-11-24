#pragma once
#include <utility>

class ContinuousFunction
{
  public:
    ContinuousFunction(
        float lower_bound
      , float peak
      , float upper_bound)
    : lower_bound_(lower_bound)
    , upper_bound_(upper_bound)
    , peak_(peak)
    , is_bound_(false)
    {}

    ContinuousFunction(float bound, float peak)
    : is_bound_(true)
    {
        peak_ = peak;
        lower_bound_ = (bound < peak) ?bound :peak;
        upper_bound_ = (bound > peak) ?bound :peak;
    }

    float get_value(float input) const
    {
        if(!is_bound_)
        {
            if(input < lower_bound_ || input > upper_bound_)
                return 0.0;
            else if(input <= peak_)
                return (input-lower_bound_) / (peak_-lower_bound_);
            else  // input > peak_
                return (upper_bound_-input) / (upper_bound_-peak_);
        }
        else if(lower_bound_ == peak_) 
        // lower_bound_ doesn't exist => left most function
        {
            if(input > upper_bound_)
                return 0.0;
            else if(input <= peak_)
                return 1.0;
            else
                return (upper_bound_-input) / (upper_bound_-peak_);
        }
        else 
        // upper_bound_ doesn't exist => right most function
        {
            if(input < lower_bound_)
                return 0.0;
            else if(input >= peak_)
                return 1.0;
            else
                return (input-lower_bound_) / (peak_-lower_bound_);
        }
    }

  private:
    float lower_bound_;
    float upper_bound_;
    float peak_;
    bool is_bound_;
};

template <int NUM_SAMPLE, int ACC_BOUND>
class SampledFunction
{
  public:
    // init all sample to 0
    SampledFunction()
    : sample(NUM_SAMPLE+1)
    , gap((float)(2*ACC_BOUND)/(float)NUM_SAMPLE)
    {
        float position = (float)(-ACC_BOUND);
        for(auto &i : sample)
        {
            i.first = position;
            i.second = 0;
            position+=gap;
        }
    }

    SampledFunction(
        float bound
      , float peak)
    : sample(NUM_SAMPLE+1)
    , gap((float)(2*ACC_BOUND)/(float)NUM_SAMPLE)
    {
        float position = (float)(-ACC_BOUND);
        for(auto &i : sample)
        {
            i.first = position;

            if(bound < peak) // right most
                if(position < bound)
                    i.second = 0.0;
                else if(position < peak)
                    i.second = (position-bound) / (peak-bound);
                else
                    i.second = 1.0;
            else // left most
                if(position > bound)
                    i.second = 0.0;
                else if(position > peak)
                    i.second = (bound-position) / (bound-peak);
                else
                    i.second = 1.0;

            position+=gap;
        }
    }

    SampledFunction(
        float lower_bound
      , float peak
      , float upper_bound)
    : sample(NUM_SAMPLE+1)
    , gap((float)(2*ACC_BOUND)/(float)NUM_SAMPLE)
    {
        float position = (float)(-ACC_BOUND);
        for(auto &i : sample)
        {
            i.first = position;

            if(position < lower_bound || position > upper_bound)
                i.second = 0.0;
            else if(position < peak)
                i.second = (position-lower_bound) / (peak-lower_bound);
            else // position is between peak and upper_bound
                i.second = (upper_bound-position) / (upper_bound-peak);

            position+=gap;
        }
    }

    // get the max of fuzzy set *this and func
    void 
    fuzzy_max(const SampledFunction<NUM_SAMPLE, ACC_BOUND> &func)
    {
        for(auto i=0; i<NUM_SAMPLE; i++)
            sample[i].second = 
                std::max(sample[i].second, func.sample[i].second);
    }

    // set the upper bound of the sampled fuzzy set
    SampledFunction<NUM_SAMPLE, ACC_BOUND> 
    bound(float bound) const
    {
        SampledFunction<NUM_SAMPLE, ACC_BOUND> output(*this);
        for(auto &i : output.sample)
            if(i.second > bound)
                i.second = bound;
        return output;
    }

    float defuzzification() const
    {
        // get centroid
        float weighted_sum = 0;
        float fuzzy_sum = 0;
        for(auto i : sample)
        {
            weighted_sum += i.first * i.second; 
            fuzzy_sum += i.second;
        }

        return weighted_sum / fuzzy_sum;
    }

    // sample point of the function
    std::vector<std::pair<float,float>> sample;
    float gap;
};
