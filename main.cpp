#include <iostream>
#include <cmath>
#include <iomanip>
#include <random>

#include "fuzzy.hpp"

// -------------------
// tunable parameters
// -------------------
#define DEBUG 1
#define TRAVEL_TIME 1000 // seconds
// Chance of front car leaves or other car enter
#define CHANCE_OF_DISTANCE_JUMP 0 // percent
// chance of front car change acceleration
#define CHANCE_OF_ACC_CHANGE 0 // percent


// ---------------------
// untuable parameters
// ---------------------
// These parameters in defined in spec, you should not 
// modified it.
#define INIT_SPEED_FRONT 30 // 30(m/s) = 108(km/hr)
#define INIT_SPEED_ME 15 // 15(m/s) = 54(km/hr)
#define INIT_DISTANCE 100 // m
#define INIT_ACC_FRONT 0 // m/s^2
#define INIT_ACC_ME 0 
#define SPEED_RELATIVE (speed_front-speed_me)
#define ACC_RELATIVE (acc_front-acc_me)
#define IDEAL_DISTANCE (speed_me*1.5) // seconds travel
#define CAR_LENGTH 5 // m


int main()
{
    // -------------------
    // init all parameters
    // -------------------
    float speed_front = INIT_SPEED_FRONT;
    float speed_me = INIT_SPEED_ME;
    float acc_front = INIT_ACC_FRONT;
    float acc_me = INIT_ACC_ME;
    float distance = INIT_DISTANCE;
    // evaluation parameters
    int num_collistion = 0;
    float sum_acc_me_square = 0;
    float sum_distance_ratio_square = 0;
    // random event relative
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 100);
    int rdm_number;
    int acc_duration;
    // fuzzy controller
    Fuzzy fuzzy_control;

    // -------------------
    // test drive begin
    // -------------------
    #if DEBUG 
        std::cout << "[test drive begin]" << std::endl;
    #endif
    for(int i=0; i<TRAVEL_TIME; i++)
    {
        #if DEBUG 
            std::cout << "[" << i << "]\n";
            std::cout << "relative speed: " 
                      << std::fixed << std::setprecision(3)
                      << SPEED_RELATIVE << "\n" 

                      << "distance ratio: "
                      << std::fixed << std::setprecision(3)
                      << distance/IDEAL_DISTANCE << "\n" 

                      << "acc_me: "
                      << std::fixed << std::setprecision(3)
                      << acc_me << "\n"

                      << "acc_front: "
                      << std::fixed << std::setprecision(3)
                      << acc_front << "\n"

                      << "speed_me: "
                      << std::fixed << std::setprecision(3)
                      << speed_me << "\n"

                      << "speed_front: "
                      << std::fixed << std::setprecision(3)
                      << speed_front << "\n";
        #endif

        // --------------
        // random event
        // --------------
        // - front car leaves or other car enter (gain/loss large
        // distance)
        rdm_number = dis(gen);
        if(rdm_number < CHANCE_OF_DISTANCE_JUMP)
            if(i%2 == 0) 
            {
                // front car leaves
                distance += (rdm_number%5+1) * CAR_LENGTH;
                #if DEBUG
                    std::cout << "front_car_leaves\t"; 
                #endif
            }
            else if(i%2 == 1 && distance > CAR_LENGTH)
            {
                // other car cut
                distance -= CAR_LENGTH;
                acc_front = 0;
                speed_front--;
                #if DEBUG
                    std::cout << "other_car_enter\t"; 
                #endif
            }
        // - front car speed up or slow down (change acc_front)
        if(acc_duration <= 0 && rdm_number < CHANCE_OF_ACC_CHANGE)
        {
            acc_duration = (rdm_number%20+1);
            acc_front = (rdm_number%2 == 0) ?1.0 :-1.0;
            #if DEBUG
                std::cout << "acceleration: " << acc_front; 
            #endif
        } 
        else
            acc_duration--;

        // -------------------------------------------------------
        // fuzzy controller do some calculation and output result
        // -------------------------------------------------------
        acc_me = fuzzy_control(
            SPEED_RELATIVE
          , distance/IDEAL_DISTANCE);
        
        // ----------------------
        // update all parameters
        // ----------------------
        speed_front += acc_front;
        speed_me += acc_me;
        // uniform acceleration: s = v0*t + (1/2)*a*t^2
        distance += SPEED_RELATIVE + 0.5*ACC_RELATIVE;

        // ----------------------------I
        // update evaluation parameters
        // ----------------------------
        sum_acc_me_square += pow(acc_me, 2);
        sum_distance_ratio_square += pow((1-distance/IDEAL_DISTANCE), 2);
        // check collision
        if(distance <= 0)
        {
            distance = 0;
            speed_me = 0;
            acc_me = 0;
            num_collistion++;
            std::cout << "\touch!!\t";  //debug
        }

        #if DEBUG 
            std::cout << std::endl;
        #endif
    }
    #if DEBUG 
        std::cout << "[test drive end]\n" << std::endl;
    #endif

    // ---------------------------
    // evaluate test drive result
    // ---------------------------
    // 1. distance handling
    std::cout << "Disstance handling "
              << "(the accumulated square difference)\n"
              << std::fixed << std::setprecision(3)
              << sum_distance_ratio_square 
              << std::endl;
    // 2. comfortness
    std::cout << "Comfortness (accumulated acceleration square)\n" 
              << std::fixed << std::setprecision(3)
              << sum_acc_me_square 
              << std::endl;
    // 3. safeness
    std::cout << "Safeness " 
              << "(#collision/seconds)\n" 
              << std::fixed << std::setprecision(3)
              << num_collistion / (float)TRAVEL_TIME 
              << std::endl;

    return 0;
}
