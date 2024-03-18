#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/target.hpp>

/*****************************************************************************/
// Std:

#include <string>

/*****************************************************************************/
// Defines:
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

    typedef enum {
        TARGET_TYPE_NONE = 0,
        TARGET_TYPE_CABLE = 1
    } target_type_t;

} // namespace adapters
} // namespace iii_drone

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

    /**
     * @brief Target adapter class.
     */
    class TargetAdapter {
    public:
        /**
         * @brief Default constructor
         */
        TargetAdapter();

        /**
         * @brief Copy constructor
         * 
         * @param other Other target adapter
         */
        TargetAdapter(const TargetAdapter & other);

        /**
         * @brief Constructor from msg.
         * 
         * @param msg Target message
         */
        TargetAdapter(const iii_drone_interfaces::msg::Target & msg);

        /**
         * @brief Constructor from members.
         * 
         * @param target_type Target type
         * @param target_id Target id
         * @param reference_frame_id Reference frame id
         * @param target_transform Target transform
         */
        TargetAdapter(
            target_type_t target_type,
            int target_id,
            const std::string & reference_frame_id,
            const iii_drone::types::transform_matrix_t & target_transform
        );

        /**
         * @brief Destructor
         */
        ~TargetAdapter();

        /**
         * @brief Converts to a target message.
         * 
         * @param target Target
         * 
         * @return Target message
         */
        iii_drone_interfaces::msg::Target ToMsg() const;

        /**
         * @brief Gets the target type.
         * 
         * @return Target type
         */
        target_type_t target_type() const;

        /**
         * @brief Gets the target id.
         * 
         * @return Target id
         */
        int target_id() const;

        /**
         * @brief Gets the reference frame id.
         * 
         * @return Reference frame id
         */
        std::string reference_frame_id() const;

        /**
         * @brief Gets the target transform.
         * 
         * @return Target transform
         */
        iii_drone::types::transform_matrix_t target_transform() const;

        /**
         * @brief Assignment operator.
         * 
         * @param other Other target adapter
         */
        TargetAdapter & operator=(const TargetAdapter & other);

    private:
        /**
         * @brief Target type.
         */
        target_type_t target_type_;

        /**
         * @brief Target id.
         */
        int target_id_;

        /**
         * @brief Reference frame id.
         */
        std::string reference_frame_id_;

        /**
         * @brief Target transform.
         */
        iii_drone::types::transform_matrix_t target_transform_;

    };

} // namespace adapters
} // namespace iii_drone