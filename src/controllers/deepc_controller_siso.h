#ifndef DEEPC_CONTROLLER_SISO_H
#define DEEPC_CONTROLLER_SISO_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <functional>
#include <vector>
//#include "data_importer_and_resampler.h"
#include "DeePCpp/controller.h"
#include "data_handling/data_importer_and_resampler.h"

class DeepCControllerSISO {
public:
    explicit DeepCControllerSISO(const YAML::Node& config);

    void update(const Eigen::VectorXd &input_vector, const Eigen::VectorXd &output_vector);
    Eigen::VectorXd computeOptimalControlInput();
    void setForceTarget(double force_target);

private:
    Controller deepc_controller_;
    double force_target_ = 0.0;  // example default target
    int target_size_;
};

#endif // DEEPC_CONTROLLER_SISO_H
