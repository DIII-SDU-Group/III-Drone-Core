#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <string>
#include <memory>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/token.hpp>
#include <iii_drone_core/utils/atomic.hpp>

#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/control/reference.hpp>

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace maneuver {

    using ReferenceCallback = std::function<Reference(const State &)>;

    typedef struct reference_callback_struct {
        utils::Atomic<ReferenceCallback> callback;
        utils::Atomic<std::string> reference_provider_name;

        reference_callback_struct(
            ReferenceCallback callback = nullptr,
            const std::string &reference_provider_name = ""
        ) {
            set(callback, reference_provider_name);
        }

        Reference operator()(const State &state) {
            if (*callback) {
                return (*callback)(state);
            } else {
                throw std::runtime_error("ReferenceCallbackStruct(): Reference callback is not set.");
            }
        }

        void set(ReferenceCallback callback, const std::string &reference_provider_name) {
            this->callback = callback;
            this->reference_provider_name = reference_provider_name;
        }

        typedef std::shared_ptr<reference_callback_struct> SharedPtr;

    } ReferenceCallbackStruct;

    typedef utils::Token<ReferenceCallbackStruct> ReferenceCallbackToken;

} // namespace maneuver
} // namespace control
} // namespace iii_drone_core