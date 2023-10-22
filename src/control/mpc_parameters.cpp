/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/control/mpc_parameters.hpp"

using namespace iii_drone::control;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

MPCParameters::MPCParameters(
    const vector_t & v_max,
    const vector_t & a_max,
    const vector_t & wp,
    const vector_t & wv,
    const vector_t & wa,
    const vector_t & wj
) : v_max_(v_max),
a_max_(a_max),
wp_(wp),
wv_(wv),
wa_(wa),
wj_(wj) { }

const vector_t & MPCParameters::v_max() const {

    std::shared_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return v_max_;

}

vector_t & MPCParameters::v_max() {

    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return v_max_;

}

const vector_t & MPCParameters::a_max() const {

    std::shared_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return a_max_;

}

vector_t & MPCParameters::a_max() {

    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return a_max_;

}

const vector_t & MPCParameters::wp() const {

    std::shared_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return wp_;

}

vector_t & MPCParameters::wp() {

    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return wp_;

}

const vector_t & MPCParameters::wv() const {

    std::shared_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return wv_;

}

vector_t & MPCParameters::wv() {

    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return wv_;

}

const vector_t & MPCParameters::wa() const {

    std::shared_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return wa_;

}

vector_t & MPCParameters::wa() {

    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return wa_;

}

const vector_t & MPCParameters::wj() const {

    std::shared_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return wj_;

}

vector_t & MPCParameters::wj() {

    std::unique_lock<std::shared_mutex> lock(parameters_mutex_);
    
    return wj_;

}