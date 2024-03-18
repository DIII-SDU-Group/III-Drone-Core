/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/flight_control_requests.hpp>

using namespace iii_drone::control;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

bool flight_control_request_t::operator==(const flight_control_request_t & rhs) const {

    return action_id == rhs.action_id && request_type == rhs.request_type && 
        request_params == rhs.request_params;

}

void flight_control_request_t::operator=(const flight_control_request_t & rhs) {

    action_id = rhs.action_id;
    request_type = rhs.request_type;
    request_params = rhs.request_params;

}

bool flight_control_request_reply_t::operator==(const flight_control_request_reply_t & rhs) const {

    return action_id == rhs.action_id && reply_type == rhs.reply_type;

}

void flight_control_request_reply_t::operator=(const flight_control_request_reply_t & rhs) {

    action_id = rhs.action_id;
    reply_type = rhs.reply_type;

}