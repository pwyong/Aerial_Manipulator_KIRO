#include "propulsion_controller/propulsion_control.hpp"

namespace platform_control
{
    PropulsionControl::PropulsionControl()
    {

        //allocation_matrix_ << 0,0,0,0,0,0

        pinv_allocation_matrix_=allocation_matrix_.completeOrthogonalDecomposition().pseudoInverse();
    }

    void PropulsionControl::control_allocation()
    {
        
    }

    void PropulsionControl::flight_control(vector<double> cur_position, vector<double> cur_attitude)
    {
    }
}