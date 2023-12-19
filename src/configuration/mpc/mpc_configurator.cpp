/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/configuration/mpc/mpc_configurator.hpp"

using namespace iii_drone::configuration;
using namespace iii_drone::control;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

MPCConfigurator::MPCConfigurator(
    rclcpp::Node *node,
    std::string parameter_prefix,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node,
        std::bind(
            &MPCConfigurator::updateMPCParametersObjectCallback,
            this,
            std::placeholders::_1
        )
    ),
    parameter_prefix_(parameter_prefix),
    after_parameter_change_callback_(after_parameter_change_callback)
{

    declareMPCParameters();
    initMPCParametersObject();

}

MPCConfigurator::MPCConfigurator(
    rclcpp::Node *node, 
    const rclcpp::QoS & qos,
    std::string parameter_prefix,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node, 
        qos,
        std::bind(
            &MPCConfigurator::updateMPCParametersObjectCallback,
            this,
            std::placeholders::_1
        )
    ),
    parameter_prefix_(parameter_prefix),
    after_parameter_change_callback_(after_parameter_change_callback)
{

    declareMPCParameters();
    initMPCParametersObject();

}

const std::shared_ptr<const MPCParameters> MPCConfigurator::mpc_parameters() const {
    
    return mpc_parameters_;

}

void MPCConfigurator::declareMPCParameters() {

    DeclareParameter<bool>("/control/MPC/MPC_use_state_feedback");
    DeclareParameter<int>("/control/MPC/MPC_N");
    DeclareParameter<float>("/control/MPC/dt");

    DeclareParameter<float>(parameter_prefix_ + "vx_max");
    DeclareParameter<float>(parameter_prefix_ + "vy_max");
    DeclareParameter<float>(parameter_prefix_ + "vz_max");

    DeclareParameter<float>(parameter_prefix_ + "ax_max");
    DeclareParameter<float>(parameter_prefix_ + "ay_max");
    DeclareParameter<float>(parameter_prefix_ + "az_max");

    DeclareParameter<float>(parameter_prefix_ + "wx");
    DeclareParameter<float>(parameter_prefix_ + "wy");
    DeclareParameter<float>(parameter_prefix_ + "wz");

    DeclareParameter<float>(parameter_prefix_ + "wvx");
    DeclareParameter<float>(parameter_prefix_ + "wvy");
    DeclareParameter<float>(parameter_prefix_ + "wvz");

    DeclareParameter<float>(parameter_prefix_ + "wax");
    DeclareParameter<float>(parameter_prefix_ + "way");
    DeclareParameter<float>(parameter_prefix_ + "waz");

    DeclareParameter<float>(parameter_prefix_ + "wjx");
    DeclareParameter<float>(parameter_prefix_ + "wjy");
    DeclareParameter<float>(parameter_prefix_ + "wjz");

}

void MPCConfigurator::initMPCParametersObject() {

    bool use_state_feedback = GetParameter("/control/MPC/MPC_use_state_feedback").as_bool();
    int N = GetParameter("/control/MPC/MPC_N").as_int();
    double dt = GetParameter("/control/MPC/dt").as_double();

    vector_t v_max = {
        (float)GetParameter(parameter_prefix_ + "vx_max").as_double(),
        (float)GetParameter(parameter_prefix_ + "vy_max").as_double(),
        (float)GetParameter(parameter_prefix_ + "vz_max").as_double()
    };

    vector_t a_max = {
        (float)GetParameter(parameter_prefix_ + "ax_max").as_double(),
        (float)GetParameter(parameter_prefix_ + "ay_max").as_double(),
        (float)GetParameter(parameter_prefix_ + "az_max").as_double()
    };

    vector_t wp = {
        (float)GetParameter(parameter_prefix_ + "wx").as_double(),
        (float)GetParameter(parameter_prefix_ + "wy").as_double(),
        (float)GetParameter(parameter_prefix_ + "wz").as_double()
    };

    vector_t wv = {
        (float)GetParameter(parameter_prefix_ + "wvx").as_double(),
        (float)GetParameter(parameter_prefix_ + "wvy").as_double(),
        (float)GetParameter(parameter_prefix_ + "wvz").as_double()
    };

    vector_t wa = {
        (float)GetParameter(parameter_prefix_ + "wax").as_double(),
        (float)GetParameter(parameter_prefix_ + "way").as_double(),
        (float)GetParameter(parameter_prefix_ + "waz").as_double()
    };

    vector_t wj = {
        (float)GetParameter(parameter_prefix_ + "wjx").as_double(),
        (float)GetParameter(parameter_prefix_ + "wjy").as_double(),
        (float)GetParameter(parameter_prefix_ + "wjz").as_double()
    };

    mpc_parameters_ = std::make_shared<MPCParameters>(
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

    mpc_parameters_initialized_ = true;

}

void MPCConfigurator::updateMPCParametersObjectCallback(const rclcpp::Parameter & parameter) {

    if (mpc_parameters_initialized_) {

        std::string parameter_name = parameter.get_name();

        if (
            parameter_name == parameter_prefix_ + "vx_max" ||
            parameter_name == parameter_prefix_ + "vy_max" ||
            parameter_name == parameter_prefix_ + "vz_max"
        ) {

            vector_t v_max = {
                (float)GetParameter(parameter_prefix_ + "vx_max").as_double(),
                (float)GetParameter(parameter_prefix_ + "vy_max").as_double(),
                (float)GetParameter(parameter_prefix_ + "vz_max").as_double()
            };

            mpc_parameters_->v_max() = v_max;

        } else if (
            parameter_name == parameter_prefix_ + "ax_max" ||
            parameter_name == parameter_prefix_ + "ay_max" ||
            parameter_name == parameter_prefix_ + "az_max"
        ) {

            vector_t a_max = {
                (float)GetParameter(parameter_prefix_ + "ax_max").as_double(),
                (float)GetParameter(parameter_prefix_ + "ay_max").as_double(),
                (float)GetParameter(parameter_prefix_ + "az_max").as_double()
            };

            mpc_parameters_->a_max() = a_max;

        } else if (
            parameter_name == parameter_prefix_ + "wx" ||
            parameter_name == parameter_prefix_ + "wy" ||
            parameter_name == parameter_prefix_ + "wz"
        ) {

            vector_t wp = {
                (float)GetParameter(parameter_prefix_ + "wx").as_double(),
                (float)GetParameter(parameter_prefix_ + "wy").as_double(),
                (float)GetParameter(parameter_prefix_ + "wz").as_double()
            };

            mpc_parameters_->wp() = wp;

        } else if (
            parameter_name == parameter_prefix_ + "wvx" ||
            parameter_name == parameter_prefix_ + "wvy" ||
            parameter_name == parameter_prefix_ + "wvz"
        ) {

            vector_t wv = {
                (float)GetParameter(parameter_prefix_ + "wvx").as_double(),
                (float)GetParameter(parameter_prefix_ + "wvy").as_double(),
                (float)GetParameter(parameter_prefix_ + "wvz").as_double()
            };

            mpc_parameters_->wv() = wv;

        } else if (
            parameter_name == parameter_prefix_ + "wax" ||
            parameter_name == parameter_prefix_ + "way" ||
            parameter_name == parameter_prefix_ + "waz"
        ) {

            vector_t wa = {
                (float)GetParameter(parameter_prefix_ + "wax").as_double(),
                (float)GetParameter(parameter_prefix_ + "way").as_double(),
                (float)GetParameter(parameter_prefix_ + "waz").as_double()
            };

            mpc_parameters_->wa() = wa;

        } else if (
            parameter_name == parameter_prefix_ + "wjx" ||
            parameter_name == parameter_prefix_ + "wjy" ||
            parameter_name == parameter_prefix_ + "wjz"
        ) {

            vector_t wj = {
                (float)GetParameter(parameter_prefix_ + "wjx").as_double(),
                (float)GetParameter(parameter_prefix_ + "wjy").as_double(),
                (float)GetParameter(parameter_prefix_ + "wjz").as_double()
            };

            mpc_parameters_->wj() = wj;

        }
    }

    if (after_parameter_change_callback_ != nullptr) {

        after_parameter_change_callback_(parameter);

    }
}