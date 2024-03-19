/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/control/trajectory_generator_node/trajectory_generator_node_configurator.hpp"

using namespace iii_drone::control::trajectory_generator_node;
using namespace iii_drone::control;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectoryGeneratorNodeConfigurator::TrajectoryGeneratorNodeConfigurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node,
        std::bind(
            &TrajectoryGeneratorNodeConfigurator::updateMPCParametersObjectsCallback,
            this,
            std::placeholders::_1
        )
    ),
    after_parameter_change_callback_(after_parameter_change_callback)
{

    declareMPCParameters();
    initMPCParametersObjects();

}

TrajectoryGeneratorNodeConfigurator::TrajectoryGeneratorNodeConfigurator(
    rclcpp::Node *node, 
    const rclcpp::QoS & qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node, 
        qos,
        std::bind(
            &TrajectoryGeneratorNodeConfigurator::updateMPCParametersObjectsCallback,
            this,
            std::placeholders::_1
        )
    ),
    after_parameter_change_callback_(after_parameter_change_callback)
{

    declareMPCParameters();
    initMPCParametersObjects();

}

bool TrajectoryGeneratorNodeConfigurator::MPC_use_state_feedback() const {

    return GetParameter("/control/MPC/MPC_use_state_feedback").as_bool();

}

int TrajectoryGeneratorNodeConfigurator::MPC_N() const {

    return GetParameter("/control/MPC/MPC_N").as_int();

}

double TrajectoryGeneratorNodeConfigurator::dt() const {

    return GetParameter("/control/MPC/dt").as_double();

}

std::string TrajectoryGeneratorNodeConfigurator::world_frame_id() const {

    return GetParameter("/tf/world_frame_id").as_string();

}

std::string TrajectoryGeneratorNodeConfigurator::drone_frame_id() const {

    return GetParameter("/tf/drone_frame_id").as_string();

}

const std::shared_ptr<MPCParameters> TrajectoryGeneratorNodeConfigurator::positional_mpc_parameters() const {
    
    return positional_mpc_parameters_;

}

const std::shared_ptr<MPCParameters> TrajectoryGeneratorNodeConfigurator::cable_landing_mpc_parameters() const {
    
    return cable_landing_mpc_parameters_;

}

const std::shared_ptr<MPCParameters> TrajectoryGeneratorNodeConfigurator::cable_takeoff_mpc_parameters() const {
    
    return cable_takeoff_mpc_parameters_;

}

void TrajectoryGeneratorNodeConfigurator::declareMPCParameters() {

    DeclareParameter<bool>("/control/MPC/MPC_use_state_feedback");
    DeclareParameter<int>("/control/MPC/MPC_N");
    DeclareParameter<float>("/control/MPC/dt");

    DeclareParameter<std::string>("/tf/world_frame_id");
    DeclareParameter<std::string>("/tf/drone_frame_id");

    auto declareMPCParametersWithPrefix = [this](std::string prefix) {

        DeclareParameter<float>(prefix + "vx_max");
        DeclareParameter<float>(prefix + "vy_max");
        DeclareParameter<float>(prefix + "vz_max");

        DeclareParameter<float>(prefix + "ax_max");
        DeclareParameter<float>(prefix + "ay_max");
        DeclareParameter<float>(prefix + "az_max");

        DeclareParameter<float>(prefix + "wx");
        DeclareParameter<float>(prefix + "wy");
        DeclareParameter<float>(prefix + "wz");

        DeclareParameter<float>(prefix + "wvx");
        DeclareParameter<float>(prefix + "wvy");
        DeclareParameter<float>(prefix + "wvz");

        DeclareParameter<float>(prefix + "wax");
        DeclareParameter<float>(prefix + "way");
        DeclareParameter<float>(prefix + "waz");

        DeclareParameter<float>(prefix + "wjx");
        DeclareParameter<float>(prefix + "wjy");
        DeclareParameter<float>(prefix + "wjz");

    };

    declareMPCParametersWithPrefix(positional_mpc_parameter_name_prefix);
    declareMPCParametersWithPrefix(cable_landing_mpc_parameter_name_prefix);
    declareMPCParametersWithPrefix(cable_takeoff_mpc_parameter_name_prefix);

}

void TrajectoryGeneratorNodeConfigurator::initMPCParametersObjects() {

    auto initMPCParametersObjectWithPrefix = [this](std::string parameter_prefix) {

        bool use_state_feedback = GetParameter("/control/MPC/MPC_use_state_feedback").as_bool();
        int N = GetParameter("/control/MPC/MPC_N").as_int();
        double dt = GetParameter("/control/MPC/dt").as_double();

        vector_t v_max = {
            (float)GetParameter(parameter_prefix + "vx_max").as_double(),
            (float)GetParameter(parameter_prefix + "vy_max").as_double(),
            (float)GetParameter(parameter_prefix + "vz_max").as_double()
        };

        vector_t a_max = {
            (float)GetParameter(parameter_prefix + "ax_max").as_double(),
            (float)GetParameter(parameter_prefix + "ay_max").as_double(),
            (float)GetParameter(parameter_prefix + "az_max").as_double()
        };

        vector_t wp = {
            (float)GetParameter(parameter_prefix + "wx").as_double(),
            (float)GetParameter(parameter_prefix + "wy").as_double(),
            (float)GetParameter(parameter_prefix + "wz").as_double()
        };

        vector_t wv = {
            (float)GetParameter(parameter_prefix + "wvx").as_double(),
            (float)GetParameter(parameter_prefix + "wvy").as_double(),
            (float)GetParameter(parameter_prefix + "wvz").as_double()
        };

        vector_t wa = {
            (float)GetParameter(parameter_prefix + "wax").as_double(),
            (float)GetParameter(parameter_prefix + "way").as_double(),
            (float)GetParameter(parameter_prefix + "waz").as_double()
        };

        vector_t wj = {
            (float)GetParameter(parameter_prefix + "wjx").as_double(),
            (float)GetParameter(parameter_prefix + "wjy").as_double(),
            (float)GetParameter(parameter_prefix + "wjz").as_double()
        };

        std::shared_ptr<MPCParameters> params = std::make_shared<MPCParameters>(
            use_state_feedback,
            N,
            dt,
            v_max,
            a_max,
            wp,
            wv,
            wa,
            wj
        );

        return params;

    };

    positional_mpc_parameters_ = initMPCParametersObjectWithPrefix(positional_mpc_parameter_name_prefix);
    cable_landing_mpc_parameters_ = initMPCParametersObjectWithPrefix(cable_landing_mpc_parameter_name_prefix);
    cable_takeoff_mpc_parameters_ = initMPCParametersObjectWithPrefix(cable_takeoff_mpc_parameter_name_prefix);

    mpc_parameters_initialized_ = true;

}

void TrajectoryGeneratorNodeConfigurator::updateMPCParametersObjectsCallback(const rclcpp::Parameter & parameter) {

    auto updateMPCParametersObjectWithPrefix = [&](
        std::string parameter_prefix,
        std::shared_ptr<MPCParameters> & mpc_parameters
    ) {

        if (mpc_parameters_initialized_) {

            std::string parameter_name = parameter.get_name();

            if (
                parameter_name == parameter_prefix + "vx_max" ||
                parameter_name == parameter_prefix + "vy_max" ||
                parameter_name == parameter_prefix + "vz_max"
            ) {

                vector_t v_max = {
                    (float)GetParameter(parameter_prefix + "vx_max").as_double(),
                    (float)GetParameter(parameter_prefix + "vy_max").as_double(),
                    (float)GetParameter(parameter_prefix + "vz_max").as_double()
                };

                mpc_parameters->v_max() = v_max;

            } else if (
                parameter_name == parameter_prefix + "ax_max" ||
                parameter_name == parameter_prefix + "ay_max" ||
                parameter_name == parameter_prefix + "az_max"
            ) {

                vector_t a_max = {
                    (float)GetParameter(parameter_prefix + "ax_max").as_double(),
                    (float)GetParameter(parameter_prefix + "ay_max").as_double(),
                    (float)GetParameter(parameter_prefix + "az_max").as_double()
                };

                mpc_parameters->a_max() = a_max;

            } else if (
                parameter_name == parameter_prefix + "wx" ||
                parameter_name == parameter_prefix + "wy" ||
                parameter_name == parameter_prefix + "wz"
            ) {

                vector_t wp = {
                    (float)GetParameter(parameter_prefix + "wx").as_double(),
                    (float)GetParameter(parameter_prefix + "wy").as_double(),
                    (float)GetParameter(parameter_prefix + "wz").as_double()
                };

                mpc_parameters->wp() = wp;

            } else if (
                parameter_name == parameter_prefix + "wvx" ||
                parameter_name == parameter_prefix + "wvy" ||
                parameter_name == parameter_prefix + "wvz"
            ) {

                vector_t wv = {
                    (float)GetParameter(parameter_prefix + "wvx").as_double(),
                    (float)GetParameter(parameter_prefix + "wvy").as_double(),
                    (float)GetParameter(parameter_prefix + "wvz").as_double()
                };

                mpc_parameters->wv() = wv;

            } else if (
                parameter_name == parameter_prefix + "wax" ||
                parameter_name == parameter_prefix + "way" ||
                parameter_name == parameter_prefix + "waz"
            ) {

                vector_t wa = {
                    (float)GetParameter(parameter_prefix + "wax").as_double(),
                    (float)GetParameter(parameter_prefix + "way").as_double(),
                    (float)GetParameter(parameter_prefix + "waz").as_double()
                };

                mpc_parameters->wa() = wa;

            } else if (
                parameter_name == parameter_prefix + "wjx" ||
                parameter_name == parameter_prefix + "wjy" ||
                parameter_name == parameter_prefix + "wjz"
            ) {

                vector_t wj = {
                    (float)GetParameter(parameter_prefix + "wjx").as_double(),
                    (float)GetParameter(parameter_prefix + "wjy").as_double(),
                    (float)GetParameter(parameter_prefix + "wjz").as_double()
                };

                mpc_parameters->wj() = wj;

            }
        }
    };

    updateMPCParametersObjectWithPrefix(positional_mpc_parameter_name_prefix, positional_mpc_parameters_);
    updateMPCParametersObjectWithPrefix(cable_landing_mpc_parameter_name_prefix, cable_landing_mpc_parameters_);
    updateMPCParametersObjectWithPrefix(cable_takeoff_mpc_parameter_name_prefix, cable_takeoff_mpc_parameters_);

    if (after_parameter_change_callback_ != nullptr) {

        after_parameter_change_callback_(parameter);

    }
}