#pragma once

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace control {

	/**
	 * @brief The supported MPC modes
	 */
	enum MPC_mode_t {
		positional = 0,
		cable_landing = 1,
		cable_takeoff = 2
	};

}
}
