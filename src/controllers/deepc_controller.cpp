#include "deepc_controller_siso.h"
#include <algorithm>
#include <stdexcept>
#include "data_handling/data_importer_and_resampler.h"
#include "library/utilities.h"

// Helper functions defined in this .cpp file
// todo generalize more and improve input data / output data functions

// Loads input data from the configuration using dataImporterAndResampler
static std::vector<Eigen::VectorXd> loadInputData(const YAML::Node &config)
{
    auto ParametersDataReading = config["data_importer_deepc_force"];
    if (!ParametersDataReading) {
        throw std::runtime_error("Invalid or missing 'data_importer_deepc_force' in configuration.");
    }

    double delta_T = ParametersDataReading["delta_T"].as<double>();
    bool import_all_data = ParametersDataReading["import_all_data"].as<bool>();
    size_t start_sample_index = ParametersDataReading["start_sample_index"].as<size_t>();
    size_t number_of_samples = ParametersDataReading["number_of_samples"].as<size_t>();
    std::string path_to_csv_file = ParametersDataReading["path_to_csv_file"].as<std::string>();
    path_to_csv_file = expandTilde(path_to_csv_file);

    std::vector<int> desired_layers = ParametersDataReading["layers_to_import"].as<std::vector<int>>();

    dataImporterAndResampler importer(
        delta_T,
        import_all_data,
        start_sample_index,
        number_of_samples,
        path_to_csv_file,
        desired_layers
    );

    return importer.getData({"rpm target"});
}

// Loads output data from the configuration using dataImporterAndResampler
static std::vector<Eigen::VectorXd> loadOutputData(const YAML::Node &config)
{
    auto ParametersDataReading = config["data_importer_deepc_force"];
    if (!ParametersDataReading) {
        throw std::runtime_error("Invalid or missing 'data_importer_deepc_force' in configuration.");
    }

    double delta_T = ParametersDataReading["delta_T"].as<double>();
    bool import_all_data = ParametersDataReading["import_all_data"].as<bool>();
    size_t start_sample_index = ParametersDataReading["start_sample_index"].as<size_t>();
    size_t number_of_samples = ParametersDataReading["number_of_samples"].as<size_t>();
    std::string path_to_csv_file = ParametersDataReading["path_to_csv_file"].as<std::string>();
    path_to_csv_file = expandTilde(path_to_csv_file);

    std::vector<int> desired_layers = ParametersDataReading["layers_to_import"].as<std::vector<int>>();

    dataImporterAndResampler importer(
        delta_T,
        import_all_data,
        start_sample_index,
        number_of_samples,
        path_to_csv_file,
        desired_layers
    );

    return importer.getData({"force"});
}

// Creates a constraint function using parameters from the configuration
static std::function<Eigen::VectorXd(const Eigen::VectorXd&)> createConstraintFunction(const YAML::Node &config)
{
    double min_value_rpm = config["input_constraints_rpm"]["min"].as<double>();
    double max_value_rpm = config["input_constraints_rpm"]["max"].as<double>();

    return [min_value_rpm, max_value_rpm](const Eigen::VectorXd& u) -> Eigen::VectorXd {
        Eigen::VectorXd constrained_u = u;
        for (int i = 0; i < u.size(); ++i) {
            constrained_u(i) = std::min(std::max(u(i), min_value_rpm), max_value_rpm);
        }
        return constrained_u;
    };
}

// Constructor
DeepCControllerSISO::DeepCControllerSISO(const YAML::Node& config)
: deepc_controller_(
    /* input_data        */ loadInputData(config),
    /* output_data       */ loadOutputData(config),
    /* T_ini             */ config["T_ini"].as<int>(),
    /* target_size       */ config["target_size"].as<int>(),
    /* Q_value           */ config["Q"].as<double>(),
    /* R_value           */ config["R"].as<double>(),
    /* constraint        */ createConstraintFunction(config),
    /* max_pgm_iterations*/ config["max_pgm_iterations"].as<int>(),
    /* pgm_tolerance     */ config["pgm_tolerance"].as<double>()
),
target_size_(config["target_size"].as<int>())
{
        // Print out the parameters we've just initialized
    int T_ini = config["T_ini"].as<int>();
    double Q_value = config["Q"].as<double>();
    double R_value = config["R"].as<double>();
    int max_pgm_iterations = config["max_pgm_iterations"].as<int>();
    double pgm_tolerance = config["pgm_tolerance"].as<double>();

    double min_value_rpm = config["input_constraints_rpm"]["min"].as<double>();
    double max_value_rpm = config["input_constraints_rpm"]["max"].as<double>();

    std::cout << "DeepCControllerSISO Initialized with:\n"
              << "T_ini: " << T_ini << "\n"
              << "target_size: " << target_size_ << "\n"
              << "Q_value: " << Q_value << "\n"
              << "R_value: " << R_value << "\n"
              << "max_pgm_iterations: " << max_pgm_iterations << "\n"
              << "pgm_tolerance: " << pgm_tolerance << "\n"
              << "min_value_rpm: " << min_value_rpm << "\n"
              << "max_value_rpm: " << max_value_rpm << "\n";
}

// Update the controller with new input/output measurements
void DeepCControllerSISO::update(const Eigen::VectorXd &input_vector, const Eigen::VectorXd &output_vector)
{
    deepc_controller_.update(input_vector, output_vector);
}

// Compute the optimal control input
Eigen::VectorXd DeepCControllerSISO::computeOptimalControlInput()
{
    int output_size = 1; // todo move
    if (!deepc_controller_.is_initialized()) {
        // Controller not ready
        return Eigen::VectorXd::Zero(output_size);
        std::cout << "deepc not yet initialized" << std::endl;
    }

    std::vector<Eigen::VectorXd> target(target_size_, Eigen::VectorXd::Constant(1, force_target_));

    std::vector<Eigen::VectorXd> opt_control_input_sequence = deepc_controller_.apply(target);
    if (opt_control_input_sequence.empty()) {
        return Eigen::VectorXd::Zero(output_size);
        std::cout << "deepc failed " << std::endl;
    }
    
    return opt_control_input_sequence[0];
}

// Set a new force target
void DeepCControllerSISO::setForceTarget(double force_target)
{
    force_target_ = force_target;
    //std::cout << "applied target " << force_target_ << std::endl;
}
