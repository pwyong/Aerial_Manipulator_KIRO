#include "propulsion_controller/propulsion_control.hpp"

namespace platform_control
{
    PropulsionControl::PropulsionControl()
    {
        wrench_.resize(6,1);
        thrust_.resize(8,1);
        allocation_matrix_.resize(6,8);
    }
}