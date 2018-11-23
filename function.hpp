#pragma once

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
        lower_bound_ = (bound < peak) ?bound :(float)(-1);
        upper_bound_ = (bound > peak) ?bound :(float)(-1);
    }

    float get_value(float input)
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
        else if(lower_bound_ < 0) // left bound function
        {
            if(input <= peak)
                return 1.0;
            else
                return (upper_bound_-input) / (upper_bound_-peak_);
        }
        else if(upper_bound_ < 0) // right bound function
        {
            if(input >= peak)
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

template <int NUM_SAMPLE>
class SampledFunction
{
  public:
    // init all sample to 0
    SampledFunction()
    : sample(NUM_SAMPLE)
    {}

    SampledFunction(
        float lower_bound
      , float peak
      , float upper_bound)
    {
        //TODO
    }

    // get the max of fuzzy set *this and func
    void fuzzy_max(SampledFunction<NUM_SAMPLE> func)
    {
        for(auto i=0; i<NUM_SAMPLE; i++)
            sample[i] = std::max(sample[i], func.sample[i]);
    }

    float defuzzification()
    {
        //TODO: centroid or maxima or center of maxima
    }

    // sample point of the function
    std::vector<float> sample;
};
